from hub import port, motion_sensor
import motor_pair
import motor, math
import runloop
import time
 
 
# =========================================================
 
# CONFIGURAÇÃO DOS MOTORESsaida4
 
# =========================================================
 
MOTORES = motor_pair.PAIR_1
 
# Motores das rodas
 
# Motor esquerdo = A / Motor direito = E
 
motor_pair.pair(MOTORES, port.A, port.E)
 
# Motores dos anexos
 
PORTA_ANEXO_E = port.F
 
PORTA_ANEXO_B = port.B
 
 
# =========================================================
 
# PARAR OS MOTORES
 
# =========================================================
 
async def parar():
 
    motor_pair.stop(MOTORES)
 
    await runloop.sleep_ms(50)
 
 
# =========================================================
 
# PROTECAO: TIMEOUT DOS MOVIMENTOS DAS RODAS
 
# =========================================================
 
def tempo_esgotado(inicio_ms, limite_ms):
 
    # ticks_diff evita problemas quando o relogio interno reinicia a contagem.
 
    return time.ticks_diff(time.ticks_ms(), inicio_ms) >= limite_ms
 
 
async def abortar_por_timeout(nome, limite_ms):
 
    # Para antes de abortar para nao continuar empurrando a missao.
 
    await parar()
 
    print("TIMEOUT em", nome, "- limite:", limite_ms, "ms")
 
    raise RuntimeError("Movimento interrompido por timeout: " + nome)
 
# =========================================================
 
# MOTORES DOS ANEXOS
 
# =========================================================
 
async def mover_anexo(porta, graus, velocidade, potencia=None, timeout_ms=None):
    if graus == 0:
        return
    if velocidade <= 0:
        raise ValueError("Velocidade do anexo deve ser positiva")
 
    if potencia is None:
        # Caminho antigo: nenhuma alteracao na mecanica ja calibrada.
        await motor.run_for_degrees(porta, graus, velocidade)
        return
 
    if potencia < 5 or potencia > 100:
        raise ValueError("Potencia deve estar entre 5 e 100 (%)")
 
    sentido = 1 if graus > 0 else -1
    alvo = abs(graus)
    pwm_maximo = int(potencia * 100)# 100% -> 10000
    pwm = min(pwm_maximo, max(800, int(velocidade * 5)))
    if timeout_ms is None:
        # Margem grande: a potencia pode limitar o alcance da velocidade.
        timeout_ms = max(1500, int(alvo * 3000 / velocidade) + 1000)
 
    motor.reset_relative_position(porta, 0)
    inicio_ms = time.ticks_ms()
    ultimo_avanco_ms = inicio_ms
    ultima_posicao = 0
 
    try:
        while True:
            agora_ms = time.ticks_ms()
            posicao = sentido * motor.relative_position(porta)
            restante = alvo - posicao
 
            # Tolerancia de 2 graus do eixo do motor.
            if restante <= 2:
                break
 
            if time.ticks_diff(agora_ms, inicio_ms) >= timeout_ms:
                raise RuntimeError("TIMEOUT no motor do anexo")
 
            # Protege contra anexo preso, obstaculo ou potencia insuficiente.
            if posicao >= ultima_posicao + 2:
                ultima_posicao = posicao
                ultimo_avanco_ms = agora_ms
            elif time.ticks_diff(agora_ms, ultimo_avanco_ms) >= 800:
                raise RuntimeError("Anexo sem movimento: verificar travamento")
 
            # Pede menos velocidade perto do alvo para reduzir ultrapassagem.
            zona_frenagem = max(35, int(velocidade * 0.18))
            velocidade_alvo = velocidade
            if restante < zona_frenagem:
                velocidade_alvo = max(45, int(velocidade * restante / zona_frenagem))
 
            # Regulacao PWM: aumenta ou reduz conforme a
            # velocidade observada, sem passar do teto de potencia.
            velocidade_atual = sentido * motor.velocity(porta)
            erro_velocidade = velocidade_alvo - velocidade_atual
            pwm += int(erro_velocidade)
            pwm = max(0, min(pwm_maximo, pwm))
 
            motor.set_duty_cycle(porta, int(sentido * pwm))
            await runloop.sleep_ms(20)
    finally:
        # Para tambem em erros, travamentos e timeouts.
        motor.stop(porta, stop=motor.BRAKE)
 
 
async def anexoe(graus, velocidade=150, potencia=None, timeout_ms=None):
    await mover_anexo(PORTA_ANEXO_E, graus, velocidade, potencia, timeout_ms)
 
 
async def anexob(graus, velocidade=250, potencia=None, timeout_ms=None):
    await mover_anexo(PORTA_ANEXO_B, graus, velocidade, potencia, timeout_ms)
 
 
 
# =========================================================
 
# CALCULAR TEMPO TEÓRICO
 
# =========================================================
 
def calcular_tempo(graus, velocidade=500):
 
    return int((abs(graus) / velocidade) * 1000)
 
 
# =========================================================
 
# CURVA PD COM GIROSCÓPIO
 
# =========================================================
 
async def girar(
 
    k,
 
    kp=0.38,
 
    kd=16,
 
    velocidade_max=500,
 
    velocidade_min=100,
 
    tolerancia=1,
 
    timeout_ms=None
 
):
 
    if timeout_ms is None:
 
        # Margem larga para que os giros PD calibrados nao sejam cortados cedo.
 
        timeout_ms = 2500 + int(abs(k) * 65)
 
    inicio_ms = time.ticks_ms()
 
    c = 360/360# C É UMA CONSTANTE DE ERRO PARA TER MAIS EXATIDÃO NO DIA
 
    angulo = k*c
 
    motion_sensor.reset_yaw(0)
 
    await runloop.sleep_ms(100)
 
    erro_anterior = angulo
 
    leitura_anterior = motion_sensor.tilt_angles()[0] / 10
 
    angulo_acumulado = 0
 
    estavel = 0
 
    while True:
 
        if tempo_esgotado(inicio_ms, timeout_ms):
 
            await abortar_por_timeout("girar", timeout_ms)
 
        leitura = motion_sensor.tilt_angles()[0] / 10
 
        diferenca = leitura - leitura_anterior
 
        # Corrige passagem entre +180 e -180
 
        if diferenca > 180:
 
            diferenca -= 360
 
        elif diferenca < -180:
 
            diferenca += 360
 
        angulo_acumulado += diferenca
 
        leitura_anterior = leitura
 
        erro = angulo - angulo_acumulado
 
        # Confirma chegada ao ângulo
 
        if abs(erro) <= tolerancia:
 
            estavel += 1
 
            if estavel >= 5:
 
                break
 
        else:
 
            estavel = 0
 
        # Controle PD
 
        proporcional = kp * erro
 
        derivativo = kd * (erro - erro_anterior)
 
        controle = proporcional + derivativo
 
        # Velocidade máxima
 
        if controle > velocidade_max:
 
            controle = velocidade_max
 
        elif controle < -velocidade_max:
 
            controle = -velocidade_max
 
        # Velocidade mínima
 
        if abs(controle) < velocidade_min:
 
            if controle > 0:
 
                controle = velocidade_min
 
            elif controle < 0:
 
                controle = -velocidade_min
 
        motor_pair.move_tank(
 
            MOTORES,
 
            int(-controle),
 
            int(controle)
 
        )
 
        erro_anterior = erro
 
        await runloop.sleep_ms(10)
 
    await parar()
 
 
# =========================================================
 
# GIRO RAPIDO NO PROPRIO EIXO
 
# =========================================================
 
async def girarr(k, velocidade=1000, tolerancia=3, timeout_ms=None):
 
    if timeout_ms is None:
 
        timeout_ms = 1800 + int(abs(k) * 40)
 
    inicio_ms = time.ticks_ms()
 
    motion_sensor.reset_yaw(0)
 
    await runloop.sleep_ms(50)
 
    if k == 0:
 
        return
 
    sentido = 1 if k > 0 else -1
 
    while True:
 
        if tempo_esgotado(inicio_ms, timeout_ms):
 
            await abortar_por_timeout("girarr", timeout_ms)
 
        yaw = motion_sensor.tilt_angles()[0] / 10
 
        if abs(yaw) >= abs(k) - tolerancia:
 
            break
 
        motor_pair.move_tank(
 
            MOTORES,
 
            -velocidade * sentido,
 
            velocidade * sentido
 
        )
 
        await runloop.sleep_ms(5)
 
    await parar()
 
 
# =========================================================
 
# ANDAR RETO
 
# ENCODER + PD DISTÂNCIA + DESACELERAÇÃO + PD DIREÇÃO
 
# =========================================================
 
async def andar(
 
    k,
 
    kp_dist=1,
 
    kd_dist=25,
 
    kp_reto=3,
 
    kd_reto=25,
 
    velocidade_max=600,
 
    velocidade_min=200,
 
    desaceleracao=3.0,
 
    timeout_ms=None
 
):
 
    if timeout_ms is None:
 
        timeout_ms = 3000 + int(abs(k) * 180)
 
    inicio_ms = time.ticks_ms()
 
    c = 30/33.5
 
    angulo = (45/math.pi)*k*c
 
    # Zera os encoders
 
    motor.reset_relative_position(port.A, 0)
 
    motor.reset_relative_position(port.E, 0)
 
    # Zera o giroscópio
 
    motion_sensor.reset_yaw(0)
 
    await runloop.sleep_ms(100)
 
    alvo = abs(angulo)
 
    erro_dist_anterior = alvo
 
    erro_reto_anterior = 0
 
    while True:
 
        if tempo_esgotado(inicio_ms, timeout_ms):
 
            await abortar_por_timeout("andar", timeout_ms)
 
        # =====================================================
 
        # ENCODERS
 
        # =====================================================
 
        posicao_esquerda = abs(
 
            motor.relative_position(port.A)
 
        )
 
        posicao_direita = abs(
 
            motor.relative_position(port.E)
 
        )
 
        posicao_media = (
 
            posicao_esquerda + posicao_direita
 
        ) / 2
 
        # Chegou ou ultrapassou o alvo
 
        if posicao_media >= alvo:
 
            break
 
        erro_dist = alvo - posicao_media
 
        # =====================================================
 
        # PD DE DISTÂNCIA
 
        # =====================================================
 
        proporcional_dist = kp_dist * erro_dist
 
        derivativo_dist = kd_dist * (
 
            erro_dist - erro_dist_anterior
 
        )
 
        velocidade = (
 
            proporcional_dist +
 
            derivativo_dist
 
        )
 
        # =====================================================
 
        # DESACELERAÇÃO
 
        # =====================================================
 
        velocidade_desacelerada = (
 
            erro_dist * desaceleracao
 
        )
 
        if velocidade_desacelerada < velocidade_max:
 
            velocidade = min(
 
                abs(velocidade),
 
                velocidade_desacelerada
 
            )
 
        # =====================================================
 
        # LIMITES DE VELOCIDADE
 
        # =====================================================
 
        if velocidade > velocidade_max:
 
            velocidade = velocidade_max
 
        if velocidade < velocidade_min:
 
            velocidade = velocidade_min
 
        # =====================================================
 
        # SENTIDO DO MOVIMENTO
 
        # =====================================================
 
        if angulo < 0:
 
            velocidade = -abs(velocidade)
 
        else:
 
            velocidade = abs(velocidade)
 
        # =====================================================
 
        # GIROSCÓPIO
 
        # =====================================================
 
        yaw = motion_sensor.tilt_angles()[0] / 10
 
        erro_reto = -yaw
 
        # =====================================================
 
        # PD DE DIREÇÃO
 
        # =====================================================
 
        proporcional_reto = kp_reto * erro_reto
 
        derivativo_reto = kd_reto * (
 
            erro_reto - erro_reto_anterior
 
        )
 
        correcao = (
 
            proporcional_reto +
 
            derivativo_reto
 
        )
 
        # =====================================================
 
        # CORRIGIR TRAJETÓRIA
 
        # =====================================================
 
        motor_esquerdo = velocidade - correcao
 
        motor_direito = velocidade + correcao
 
        motor_esquerdo = max(
 
            -velocidade_max,
 
            min(velocidade_max, motor_esquerdo)
 
        )
 
        motor_direito = max(
 
            -velocidade_max,
 
            min(velocidade_max, motor_direito)
 
        )
 
        motor_pair.move_tank(
 
            MOTORES,
 
            int(motor_esquerdo),
 
            int(motor_direito)
 
        )
 
        erro_dist_anterior = erro_dist
 
        erro_reto_anterior = erro_reto
 
        await runloop.sleep_ms(10)
 
    await parar()
 
 
# ADIÇÃO
 
# =========================================================
 
# ANDAR RETO
 
# ENCODER + PD DISTÂNCIA + DESACELERAÇÃO + PD DIREÇÃO
 
# =========================================================
 
async def andarl(
 
    k,
 
    kp_dist=2,
 
    kd_dist=20,
 
    kp_reto=3,
 
    kd_reto=30,
 
    velocidade_max=170,
 
    velocidade_min=100,
 
    desaceleracao=2.0,
 
    timeout_ms=None
 
):
 
    if timeout_ms is None:
 
        timeout_ms = 3000 + int(abs(k) * 250)
 
    inicio_ms = time.ticks_ms()
 
    c = 30/33.5
 
    angulo = (45/math.pi)*k*c
 
    # Zera os encoders
 
    motor.reset_relative_position(port.A, 0)
 
    motor.reset_relative_position(port.E, 0)
 
    # Zera o giroscópio
 
    motion_sensor.reset_yaw(0)
 
    await runloop.sleep_ms(100)
 
    alvo = abs(angulo)
 
    erro_dist_anterior = alvo
 
    erro_reto_anterior = 0
 
    while True:
 
        if tempo_esgotado(inicio_ms, timeout_ms):
 
            await abortar_por_timeout("andarl", timeout_ms)
 
        # =====================================================
 
        # ENCODERS
 
        # =====================================================
 
        posicao_esquerda = abs(
 
            motor.relative_position(port.A)
 
        )
 
        posicao_direita = abs(
 
            motor.relative_position(port.E)
 
        )
 
        posicao_media = (
 
            posicao_esquerda + posicao_direita
 
        ) / 2
 
        # Chegou ou ultrapassou o alvo
 
        if posicao_media >= alvo:
 
            break
 
        erro_dist = alvo - posicao_media
 
        # =====================================================
 
        # PD DE DISTÂNCIA
 
        # =====================================================
 
        proporcional_dist = kp_dist * erro_dist
 
        derivativo_dist = kd_dist * (
 
            erro_dist - erro_dist_anterior
 
        )
 
        velocidade = (
 
            proporcional_dist +
 
            derivativo_dist
 
        )
 
        # =====================================================
 
        # DESACELERAÇÃO
 
        # =====================================================
 
        velocidade_desacelerada = (
 
            erro_dist * desaceleracao
 
        )
 
        if velocidade_desacelerada < velocidade_max:
 
            velocidade = min(
 
                abs(velocidade),
 
                velocidade_desacelerada
 
            )
 
        # =====================================================
 
        # LIMITES DE VELOCIDADE
 
        # =====================================================
 
        if velocidade > velocidade_max:
 
            velocidade = velocidade_max
 
        if velocidade < velocidade_min:
 
            velocidade = velocidade_min
 
        # =====================================================
 
        # SENTIDO DO MOVIMENTO
 
        # =====================================================
 
        if angulo < 0:
 
            velocidade = -abs(velocidade)
 
        else:
 
            velocidade = abs(velocidade)
 
        # =====================================================
 
        # GIROSCÓPIO
 
        # =====================================================
 
        yaw = motion_sensor.tilt_angles()[0] / 10
 
        erro_reto = -yaw
 
        # =====================================================
 
        # PD DE DIREÇÃO
 
        # =====================================================
 
        proporcional_reto = kp_reto * erro_reto
 
        derivativo_reto = kd_reto * (
 
            erro_reto - erro_reto_anterior
 
        )
 
        correcao = (
 
            proporcional_reto +
 
            derivativo_reto
 
        )
 
        # =====================================================
 
        # CORRIGIR TRAJETÓRIA
 
        # =====================================================
 
        motor_esquerdo = velocidade - correcao
 
        motor_direito = velocidade + correcao
 
        motor_esquerdo = max(
 
            -velocidade_max,
 
            min(velocidade_max, motor_esquerdo)
 
        )
 
        motor_direito = max(
 
            -velocidade_max,
 
            min(velocidade_max, motor_direito)
 
        )
 
        motor_pair.move_tank(
 
            MOTORES,
 
            int(motor_esquerdo),
 
            int(motor_direito)
 
        )
 
        erro_dist_anterior = erro_dist
 
        erro_reto_anterior = erro_reto
 
        await runloop.sleep_ms(10)
 
    await parar()
 
