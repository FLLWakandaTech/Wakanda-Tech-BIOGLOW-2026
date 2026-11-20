# Tecnologias e Algoritmos do Robô | Wakanda Tech

**FIRST LEGO League Challenge — BIOGLOW 2026**  
**SESI Milton Santos | Camaçari, Bahia 🇧🇷**

Bem-vindos à documentação técnica do sistema de programação desenvolvido pela equipe **Wakanda Tech** para a temporada BIOGLOW 2026 da FIRST LEGO League Challenge.

Nossa programação foi desenvolvida utilizando **Python e LEGO Education SPIKE Prime**, integrando conceitos de matemática, física, engenharia de controle e desenvolvimento de software.

O sistema combina sensores, algoritmos matemáticos, controladores proporcional-derivativos (PD), calibração, controle de velocidade e mecanismos de proteção para controlar os movimentos do robô.

Esta documentação apresenta as **tecnologias e os fundamentos utilizados na base do código**, independentemente das missões executadas.

---

## 📑 Índice

1. [Plataforma e arquitetura de programação](#1-plataforma-e-arquitetura-de-programação)
2. [Sensores e aquisição de dados](#2-sensores-e-aquisição-de-dados)
3. [Controle em malha fechada e algoritmo PD](#3-controle-em-malha-fechada-e-algoritmo-pd)
4. [Matemática aplicada ao deslocamento](#4-matemática-aplicada-ao-deslocamento)
5. [Controle simultâneo de distância e direção](#5-controle-simultâneo-de-distância-e-direção)
6. [Gerenciamento de velocidade e desaceleração](#6-gerenciamento-de-velocidade-e-desaceleração)
7. [Controle angular e giros](#7-controle-angular-e-giros)
8. [Sistema de locomoção diferencial](#8-sistema-de-locomoção-diferencial)
9. [Controle dos anexos e modulação PWM](#9-controle-dos-anexos-e-modulação-pwm)
10. [Proteção, tratamento de falhas e segurança](#10-proteção-tratamento-de-falhas-e-segurança)
11. [Programação assíncrona e modularização](#11-programação-assíncrona-e-modularização)
12. [Funções e parâmetros do sistema](#12-funções-e-parâmetros-do-sistema)

---

## 1. Plataforma e arquitetura de programação

O sistema utiliza a linguagem **Python**, executada no hub LEGO Education SPIKE Prime, permitindo o desenvolvimento de algoritmos personalizados para o controle dos motores e a leitura dos sensores.

A arquitetura foi organizada em funções reutilizáveis, responsáveis por tarefas específicas, como deslocamento, rotação, acionamento de mecanismos, controle de velocidade e proteção contra falhas.

### Bibliotecas utilizadas

| Biblioteca | Aplicação |
|---|---|
| `hub.port` | Identificação das portas do hub |
| `hub.motion_sensor` | Leitura e redefinição da orientação angular |
| `motor_pair` | Controle dos motores de locomoção |
| `motor` | Controle e leitura dos motores e encoders |
| `runloop` | Execução de funções assíncronas |
| `math` | Operações matemáticas e utilização de π |
| `time` | Temporização e controle de limites de execução |

### Configuração dos motores

| Componente | Porta |
|---|---|
| Motor esquerdo | A |
| Motor direito | E |
| Motor do anexo E | F |
| Motor do anexo B | B |

Os motores de locomoção são configurados como um par por meio da biblioteca `motor_pair`, permitindo que cada motor receba comandos de velocidade independentes.

Essa configuração é fundamental para a realização de correções de trajetória e giros controlados.

---

## 2. Sensores e aquisição de dados

A programação utiliza informações dos sensores para monitorar o comportamento do robô e ajustar os comandos enviados aos motores.

### 2.1. Encoders dos motores

Os encoders registram a posição angular dos eixos dos motores, permitindo acompanhar a quantidade de rotação realizada.

O código utiliza a função:

`motor.relative_position()`

Para calcular o progresso do deslocamento, é utilizada a média dos valores absolutos das posições dos dois motores:

$$
\theta_{média}=\frac{|\theta_A|+|\theta_E|}{2}
$$

Onde:

- `θA`: posição angular do motor esquerdo.
- `θE`: posição angular do motor direito.
- `θmédia`: média das rotações registradas.

Antes de cada deslocamento, os encoders são redefinidos, estabelecendo uma nova referência de posição.

### 2.2. Sensor de orientação — Yaw

O sensor de movimento integrado ao hub permite medir a orientação angular do robô.

A programação utiliza:

`motion_sensor.tilt_angles()[0] / 10`

A divisão por dez converte a leitura utilizada pela API em graus.

O sensor é empregado para identificar desvios de direção e acompanhar o movimento angular durante os giros.

A função `motion_sensor.reset_yaw(0)` redefine a referência angular no início dos movimentos controlados.

---

## 3. Controle em malha fechada e algoritmo PD

Uma das principais tecnologias utilizadas na programação é o **controle em malha fechada**, no qual os comandos dos motores são continuamente ajustados a partir das informações fornecidas pelos sensores.

Em vez de depender exclusivamente de uma velocidade fixa, o robô calcula erros e modifica sua atuação para se aproximar do comportamento desejado.

### 3.1. Controlador Proporcional-Derivativo

O algoritmo PD utiliza duas componentes matemáticas:

**Componente proporcional (P)**

$$
P=K_p\cdot e
$$

A parcela proporcional reage à intensidade do erro atual.

**Componente derivativa (D)**

$$
D=K_d\cdot(e_{atual}-e_{anterior})
$$

A parcela derivativa considera a variação do erro entre duas leituras consecutivas.

**Equação implementada**

$$
\boxed{u=K_p\cdot e+K_d\cdot(e_{atual}-e_{anterior})}
$$

Onde:

- `u`: comando calculado pelo controlador.
- `e`: erro entre a referência desejada e a medição.
- `Kp`: ganho proporcional.
- `Kd`: ganho derivativo.

O objetivo do controle é ajustar a resposta dos motores conforme o erro identificado.

**Observação técnica:** a parcela derivativa do nosso código utiliza a diferença discreta entre erros consecutivos, sem divisão explícita pelo intervalo de tempo. Os ganhos são definidos para essa implementação.

O algoritmo empregado é **PD**, sem a componente integral característica de um controlador PID.

---

## 4. Matemática aplicada ao deslocamento

Para controlar deslocamentos lineares, utilizamos relações geométricas entre a circunferência das rodas e a rotação dos motores.

### 4.1. Circunferência da roda

A circunferência é calculada por:

$$
C=2\pi r
$$

Considerando o raio de 4 cm registrado na programação:

$$
C=2\pi\cdot4
$$

$$
\boxed{C=8\pi\text{ cm}}
$$

Isso significa que uma rotação completa da roda corresponde teoricamente a um deslocamento de aproximadamente 25,13 cm, desconsiderando deslizamentos e outras diferenças físicas.

### 4.2. Conversão entre centímetros e graus

Como uma rotação completa corresponde a 360°, podemos estabelecer a relação:

$$
\frac{360^\circ}{8\pi\text{ cm}}
$$

Portanto:

$$
\boxed{\theta=\frac{45}{\pi}\cdot d}
$$

Onde:

- `θ`: rotação teórica do motor, em graus.
- `d`: distância desejada, em centímetros.

### 4.3. Constante de calibração

Para ajustar a conversão matemática ao comportamento do robô, a programação inclui uma constante de calibração.

No código analisado:

```python
c = 30 / 33.5
angulo = (45 / math.pi) * k * c
```

A equação utilizada é:

$$
\boxed{\theta_{alvo}=\frac{45}{\pi}\cdot d\cdot c}
$$

Com:

$$
c=\frac{30}{33,5}\approx0,8955
$$

Essa constante modifica a conversão teórica entre distância e rotação, permitindo compensar diferenças identificadas na calibração.

A programação também interpreta o sinal da distância para determinar se o movimento ocorrerá para frente ou para trás.

---

## 5. Controle simultâneo de distância e direção

O sistema de deslocamento emprega **dois controladores PD dentro da mesma função**, permitindo acompanhar a distância percorrida e corrigir a orientação durante o movimento.

### 5.1. Controle PD de distância

O primeiro controlador utiliza as informações dos encoders.

O erro é calculado por:

$$
e_d=\theta_{alvo}-\theta_{média}
$$

Em seguida, o algoritmo PD calcula:

$$
u_d=K_{p,d}e_d+K_{d,d}(e_{d,atual}-e_{d,anterior})
$$

Esse resultado participa da determinação da velocidade-base aplicada ao robô.

O movimento termina quando a posição média das rodas alcança ou ultrapassa a referência estabelecida.

### 5.2. Controle PD de direção

O segundo controlador utiliza a orientação fornecida pelo sensor de movimento do hub.

Como a referência de orientação é redefinida para zero, o erro é:

$$
e_r=0-yaw
$$

Portanto:

$$
\boxed{e_r=-yaw}
$$

A correção é calculada por:

$$
u_r=K_{p,r}e_r+K_{d,r}(e_{r,atual}-e_{r,anterior})
$$

O resultado é distribuído entre os motores:

$$
v_{esquerda}=v_{base}-u_r
$$

$$
v_{direita}=v_{base}+u_r
$$

A diferença entre os comandos permite compensar desvios angulares enquanto o robô continua realizando o deslocamento.

### Integração dos controladores

Os dois controladores trabalham sobre variáveis distintas:

- **PD de distância:** acompanha o progresso do deslocamento.
- **PD de direção:** acompanha o alinhamento angular.

Essa integração permite controlar simultaneamente dois aspectos importantes da movimentação.

---

## 6. Gerenciamento de velocidade e desaceleração

A programação oferece dois perfis de deslocamento, implementados pelas funções `andar()` e `andarl()`.

Ambas utilizam a mesma arquitetura geral, mas apresentam parâmetros diferentes.

### Parâmetros padrão

| Parâmetro | `andar()` | `andarl()` |
|---|---:|---:|
| Kp de distância | 1 | 2 |
| Kd de distância | 25 | 20 |
| Kp de direção | 3 | 3 |
| Kd de direção | 25 | 30 |
| Velocidade máxima | 600 | 170 |
| Velocidade mínima | 200 | 100 |
| Desaceleração | 3.0 | 2.0 |

### 6.1. Limitação de velocidade

Os comandos são limitados de acordo com os valores máximos e mínimos definidos para cada perfil.

Isso permite configurar comportamentos diferentes sem alterar a estrutura principal do algoritmo.

### 6.2. Desaceleração proporcional à distância restante

O sistema calcula uma referência de desaceleração baseada no erro de distância:

$$
\boxed{v_{desaceleração}=e_d\cdot f}
$$

Onde:

- `ed`: distância angular restante.
- `f`: fator configurado para desaceleração.

Esse valor participa da limitação da velocidade comandada pelo controlador.

À medida que o robô se aproxima do alvo, a programação pode reduzir a velocidade solicitada, respeitando os limites mínimos e máximos estabelecidos.

A desaceleração implementada não equivale a uma redução obrigatória até velocidade zero, pois existe um limite mínimo de velocidade durante a execução.

---

## 7. Controle angular e giros

A programação possui duas estratégias de giro, implementadas pelas funções `girar()` e `girarr()`.

### 7.1. Giro com controle PD

A função `girar()` utiliza o sensor de orientação para acompanhar a rotação do robô.

O controlador calcula o erro angular:

$$
e_\theta=\theta_{alvo}-\theta_{acumulado}
$$

O comando é definido pela equação:

$$
u_\theta=K_p e_\theta+K_d(e_{\theta,atual}-e_{\theta,anterior})
$$

### Parâmetros padrão

| Parâmetro | Valor |
|---|---:|
| Kp | 0.38 |
| Kd | 16 |
| Velocidade máxima | 500 |
| Velocidade mínima | 100 |
| Tolerância | 1° |

O algoritmo exige cinco leituras consecutivas dentro da tolerância angular antes de concluir o giro.

Isso cria uma condição de confirmação da posição angular, em vez de finalizar imediatamente após uma única leitura satisfatória.

### 7.2. Acumulação angular

Uma característica importante do algoritmo é o tratamento da passagem entre `+180°` e `-180°`.

Sem essa correção, a mudança na representação angular poderia ser interpretada como uma rotação muito maior do que a realmente realizada.

O código utiliza:

```python
diferenca = leitura - leitura_anterior

if diferenca > 180:
    diferenca -= 360
elif diferenca < -180:
    diferenca += 360

angulo_acumulado += diferenca
```

Essa técnica permite acompanhar o ângulo acumulado a partir das diferenças entre leituras sucessivas.

A função também utiliza uma constante de ajuste angular, atualmente configurada como `360/360`, equivalente a 1.

### 7.3. Giro rápido

A função `girarr()` emprega uma estratégia mais direta.

Ela utiliza comandos de velocidades opostas nos motores e monitora a leitura angular até atingir o limite estabelecido.

Seus parâmetros padrão incluem:

- Velocidade: 1000.
- Tolerância angular: 3°.
- Limite de tempo calculado conforme o ângulo solicitado.

Diferentemente de `girar()`, essa função não utiliza controlador PD nem o mesmo mecanismo de acumulação angular.

---

## 8. Sistema de locomoção diferencial

O robô utiliza **locomoção diferencial**, baseada no controle independente das velocidades dos motores esquerdo e direito.

Esse princípio permite realizar diferentes tipos de movimento a partir da relação entre as velocidades das rodas.

### Movimento retilíneo

Quando os motores possuem velocidades equivalentes, o robô tende a se deslocar em linha reta.

$$
v_E=v_D
$$

### Correção de trajetória

Quando as velocidades são diferentes, ocorre uma mudança na trajetória.

$$
v_E\neq v_D
$$

### Giro no próprio eixo

Quando os motores são comandados com velocidades de mesma intensidade e sinais opostos, o robô tende a girar em torno da região central entre as rodas motrizes.

$$
v_E=-v_D
$$

A implementação utiliza `motor_pair.move_tank()` para enviar os comandos individuais aos motores.

Esse sistema permite transformar as correções calculadas pelos controladores em movimentos físicos.

---

## 9. Controle dos anexos e modulação PWM

A programação dos anexos utiliza uma função central chamada `mover_anexo()`, que permite controlar os mecanismos conectados aos motores das portas F e B.

O sistema possui dois modos de funcionamento.

### 9.1. Controle por posição e velocidade

Quando nenhum valor de potência é informado, utiliza-se:

`motor.run_for_degrees()`

Esse recurso permite solicitar um movimento angular, definindo a rotação desejada e a velocidade correspondente.

### 9.2. Controle com limitação PWM

Quando o parâmetro `potencia` é informado, o código utiliza um sistema próprio de acionamento com realimentação de velocidade e monitoramento de posição.

O PWM (*Pulse Width Modulation*, ou Modulação por Largura de Pulso) é uma técnica de controle que regula o acionamento elétrico por meio da proporção de tempo em que um sinal permanece ativo.

No código, o valor de potência é convertido para o limite numérico utilizado na API:

$$
\boxed{PWM_{máximo}=potencia\cdot100}
$$

Assim, uma configuração de 100% corresponde ao valor numérico 10.000.

**Importante:** esse valor representa um limite do comando PWM, não uma medição direta da potência mecânica desenvolvida pelo motor.

### 9.3. Realimentação de velocidade

O sistema acompanha a velocidade real do motor e a compara com a velocidade desejada.

$$
e_v=v_{desejada}-v_{medida}
$$

O comando PWM é atualizado a partir desse erro:

$$
PWM_{novo}=PWM_{anterior}+e_v
$$

Na implementação, a diferença é convertida para inteiro e o comando é limitado ao intervalo permitido.

Essa estratégia busca aproximar a velocidade observada da referência estabelecida, respeitando o teto de PWM.

### 9.4. Controle angular dos anexos

O encoder acompanha a posição do eixo do motor.

O algoritmo calcula:

$$
e_\theta=\theta_{alvo}-\theta_{atual}
$$

O movimento é considerado concluído quando a distância angular restante é igual ou inferior à tolerância de 2° utilizada no código.

### 9.5. Desaceleração próxima ao alvo

A programação identifica uma região de frenagem próxima da posição desejada.

Dentro dessa região, a velocidade-alvo é reduzida proporcionalmente ao ângulo restante, respeitando os valores mínimos programados.

O sistema também monitora a ausência de progresso angular para identificar possíveis travamentos.

O controle PWM dos anexos utiliza realimentação de velocidade e posição, mas não emprega a mesma equação PD utilizada nos movimentos das rodas.

---

## 10. Proteção, tratamento de falhas e segurança

O código apresenta mecanismos de proteção destinados a interromper movimentos quando determinadas condições anormais são detectadas.

### 10.1. Timeout

O *timeout* estabelece um tempo máximo para a realização de um movimento.

A programação utiliza `time.ticks_ms()` para acompanhar o relógio interno e `time.ticks_diff()` para calcular diferenças entre leituras.

A comparação permite verificar se o movimento ultrapassou o limite configurado.

Quando o tempo é excedido nas rotinas de locomoção, o sistema executa a parada e gera uma exceção.

### 10.2. Detecção de travamento dos anexos

No modo PWM, o algoritmo monitora a variação da posição angular.

Caso não seja detectado progresso suficiente durante 800 milissegundos, o sistema gera uma exceção informando a ausência de movimento.

Essa verificação busca identificar situações como obstáculos mecânicos, mecanismos presos ou acionamento insuficiente.

### 10.3. Frenagem dos motores

A rotina PWM dos anexos utiliza uma estrutura `try/finally`.

Isso garante que o comando de parada do motor seja executado quando a rotina termina, inclusive quando ocorre uma exceção.

O acionamento de frenagem é realizado por:

`motor.stop(porta, stop=motor.BRAKE)`

### 10.4. Validação de parâmetros

A programação também verifica determinadas entradas antes de iniciar o acionamento dos anexos.

Entre as verificações estão:

- Velocidade positiva.
- Potência entre 5% e 100% no modo PWM.
- Encerramento imediato quando o deslocamento angular solicitado é zero.

Esses recursos aumentam a capacidade do software de identificar situações inesperadas, embora não substituam verificações físicas e testes de segurança.

---

## 11. Programação assíncrona e modularização

A programação utiliza funções assíncronas definidas por `async def`, juntamente com a palavra-chave `await`.

Essa arquitetura permite organizar operações que dependem de leituras, comandos e intervalos de espera.

### Ciclos de atualização

| Sistema | Intervalo solicitado |
|---|---:|
| Deslocamento `andar()` | 10 ms |
| Deslocamento `andarl()` | 10 ms |
| Giro PD `girar()` | 10 ms |
| Giro rápido `girarr()` | 5 ms |
| Controle PWM dos anexos | 20 ms |

Os intervalos representam pausas solicitadas pelo programa, não frequências de execução rigidamente garantidas pelo hardware.

### Modularização

A base foi dividida em funções com responsabilidades específicas.

Essa organização permite:

- Reutilizar algoritmos sem reescrevê-los.
- Modificar parâmetros de controle.
- Separar a lógica de movimentação do acionamento dos anexos.
- Facilitar a leitura e a manutenção do código.
- Aplicar diferentes perfis de movimento.
- Concentrar verificações de segurança em funções próprias.

A modularização também contribui para a organização do desenvolvimento coletivo, pois estabelece interfaces claras para os principais recursos do robô.

---

## 12. Funções e parâmetros do sistema

A programação disponibiliza as seguintes funções principais:

| Função | Responsabilidade |
|---|---|
| `andar()` | Deslocamento com PD de distância, PD de direção e desaceleração |
| `andarl()` | Deslocamento com parâmetros próprios para um perfil mais lento |
| `girar()` | Giro angular com controle PD e confirmação de estabilidade |
| `girarr()` | Giro rápido com monitoramento angular |
| `mover_anexo()` | Controle de posição, velocidade e potência dos mecanismos |
| `anexoe()` | Interface para o motor de anexo da porta F |
| `anexob()` | Interface para o motor de anexo da porta B |
| `parar()` | Interrupção dos motores de locomoção |
| `tempo_esgotado()` | Verificação do tempo transcorrido |
| `abortar_por_timeout()` | Parada e interrupção por tempo excedido |
| `calcular_tempo()` | Estimativa teórica do tempo a partir de graus e velocidade |

### Integração das tecnologias

A base de programação desenvolvida pela Wakanda Tech reúne diferentes conceitos de tecnologia e engenharia:

**Programação:** Python, funções assíncronas, modularização e tratamento de exceções.

**Matemática:** geometria da circunferência, proporcionalidade, conversão de unidades, funções e cálculo de erros.

**Física e robótica:** movimento angular, deslocamento linear, sensores de orientação, encoders e locomoção diferencial.

**Controle e automação:** controladores PD, realimentação, calibração, PWM, limites de velocidade e detecção de condições anormais.

A integração desses conhecimentos permite desenvolver uma programação baseada em medições e correções, na qual os algoritmos se relacionam diretamente com o comportamento físico do robô.

**FIRST LEGO League Challenge | BIOGLOW 2026**  
**Torneio Regional — SESI Milton Santos | Camaçari, Bahia 🇧🇷**
