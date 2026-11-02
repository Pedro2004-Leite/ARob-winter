# Filtro de Batista et al. no Lab 2 (Secção 5) - apontamento

Este apontamento explica a lógica do que foi implementado, porque foi feito assim, o que foi validado e o que fica por verificar.
Os números aqui reportados saem todos de execuções dos scripts desta pasta (comandos na secção 8).

## 1. O que foi feito

Implementou-se o filtro de Kalman do paper de Batista, Silvestre e Oliveira (Int. J. Control, 2011), para o sistema nominal (2), com a estimação parcial de atitude (roll e pitch) e do bias dos 3 rate gyros.
O filtro está em Simulink, em tempo contínuo, e foi testado de duas formas.
Primeiro sobre os dados reais das experiências A a E (modelo de Replay do DevKit).
Depois sobre uma simulação com verdade conhecida, onde o erro absoluto se pode medir.

## 2. O modelo e o filtro

A atitude obedece a `Rdot = R S(w)`, com `R` a rotação do corpo para o referencial inercial.
O acelerómetro mede, em eixos do corpo, um vetor constante no referencial inercial (a gravidade): `y = R' c`.
Derivando, e usando `w_m = w + b` (gyro com bias), obtém-se `ydot = -S(w) y = -S(w_m) y - S(y) b`.
O estado é `x = [y; b]` (6 componentes), com o bias constante:

```
xdot = A(t) x,   A = [ -S(w_m)  -S(y) ]     y = C x,   C = [ I  0 ]
                     [    0       0   ]
```

O sistema é não linear, mas `A(t)` só depende de `y` e de `w_m`, que são ambos medidos.
Por isso pode tratar-se como linear variante no tempo e aplicar o Kalman-Bucy clássico:

```
xhat_dot = A xhat + K (y - C xhat),    K = P C' Th^-1
P_dot    = A P + P A' + Xi - P C' Th^-1 C P
```

`Xi` e `Th` são as intensidades do ruído de processo e de medida.
O ganho tem de ser recalculado a cada instante, porque `A(t)` muda (não é o ganho estacionário da secção 4 do guia).

Observabilidade (Teoremas 3.3 e 3.4 do paper): o sistema é observável se `y` não for constante, e a estabilidade do erro precisa de que `y` continue a variar (excitação persistente).
Com o drone parado, o bias na direção da gravidade (o gyro z) não é observável.

## 3. Implementação

Ficheiros em `labs-26-27/Lab2/Filter/`:

- `ARDroneReplay_BatistaKF.slx`: o filtro sobre os dados gravados.
- `ARDroneSim_BatistaKF.slx`: o mesmo filtro sobre uma experiência sintética com verdade conhecida.
- `run_batista_kf.m` e `run_batista_kf_sim.m`: correm cada modelo, abrem os scopes e imprimem um resumo.
- `build_batista_kf_model.m`: regenera os dois modelos (o filtro é um bloco partilhado).

Escolhas de implementação:

- **Tempo contínuo.** Segue o paper. `x` e `P` são integrados por blocos Integrator, com solver de passo fixo `ode1` a 5 ms (o período dos dados e o solver do modelo original do DevKit).
- **Unidades.** O acelerómetro vem em mg e passa a m/s^2 (x 9,81/1000). Os gyros vêm em deg/s e passam a rad/s. Os ângulos mostram-se em graus.
- **Sinais.** Neste sensor `az = -g` em repouso. Por isso `phi = atan2(-ay, -az)` e `theta = asin(ax / |y|)`. A fórmula `atan2(ay, az)` dá phi + 180 graus. O `ω_m` entra diretamente dos gyros: `gx` e `gy` têm correlação de +0,97 a +0,98 e +0,94 a +0,98 com as derivadas de phi e theta.
- **Ângulos.** Calculam-se a partir do vetor filtrado `x1_hat`, e não do acelerómetro cru.
- **Condições iniciais.** `x0 = [0, 0, -g, 0, 0, 0]` (nivelado, bias nulo) e `P0 = blkdiag(I, 0,01 I)`. Não se usa a primeira amostra, porque o primeiro pacote do `navdata` é todo zeros.

## 4. Afinação

```
Xi = blkdiag( (g*0,035)^2 * Ts * I3 ,  1e-5 * I3 )     (Ts = 5 ms)
Th = 1e3 * 0,07^2 * Ts * I3
```

- **Ruído de medida `Th`.** `0,07 m/s^2` é o desvio do acelerómetro em hover (experiência C: 0,059 a 0,077). Multiplica-se por `Ts` para passar de variância amostrada a intensidade contínua, como no `kalman_filters.m` da secção 4.
- **Fator 1000 em `Th`.** Em voo o erro do acelerómetro não é ruído branco, é vibração e aceleração linear. Por isso a medida merece menos confiança do que o ruído sugere.
- **Ruído de processo `Xi`.** O ruído dos gyros entra em `ydot` multiplicado por `y`, que tem módulo ~g, daí `(g*sigma_g)^2 * Ts` com `sigma_g = 0,035 rad/s` (hover). O `1e-5` é a intensidade do passeio aleatório do bias.
- **Como foram escolhidos.** O fator 1000 e o `1e-5` vêm de um varrimento (fator entre 1 e 1e5, intensidade do bias entre 1e-8 e 1e-4) nas experiências D e E, a minimizar o erro contra o estimador interno do drone.

Esta afinação é heurística e tem duas fraquezas assumidas.
Foi feita nos mesmos dados em que depois se avalia.
E a referência (o estimador interno do drone) não é a verdade.

## 5. Validação

### 5.1 Simulação com verdade conhecida

Roll de 8 graus a 0,30 Hz e pitch de 6 graus a 0,45 Hz (fase de 60 graus), yaw parado.
Frequências diferentes mantêm `y` a mudar de direção (condição de excitação do Teorema 3.4).
Gyros e acelerómetro saem da cinemática exata (verificado numericamente: `ydot + S(w) y` = 1e-8, contra uma derivada de ~2,6).
Ruído gaussiano de 0,07 m/s^2 no acelerómetro e 0,01 rad/s nos gyros, bias dos gyros de [2, -3, 1] deg/s (o valor da simulação do paper).
Erro depois de 10 s de transitório, contra a verdade:

| Caso | RMS phi / theta | Erro médio phi / theta | Bias estimado [deg/s] (verdadeiro [2, -3, 1]) |
|---|---|---|---|
| Por defeito | 0,05 / 0,05 graus | +0,01 / 0,00 | [2,01, -3,01, 0,96] |
| Bias de acelerómetro [-0,033, -0,166, 0] m/s^2 | 0,98 / 0,20 graus | +0,98 / -0,19 | [2,01, -3,01, 0,89] |
| Ruído x 0,01 | 0,03 / 0,03 graus | 0,00 / 0,00 | [2,00, -3,00, 0,93] |

Conclusões: o filtro está correto (o erro é ~0,05 graus e o bias é recuperado nos três eixos).
Um bias no acelerómetro produz um offset constante em phi e theta que o filtro não consegue corrigir.

### 5.2 Dados reais contra o estimador interno do drone

Nos dados reais não há verdade.
A comparação faz-se contra os `states` do drone (portas 1 e 2 do bloco de Replay), em t > 10 s e t < fim - 3 s (sem descolagem nem aterragem).
O RMS calcula-se depois de retirar o offset constante, que se reporta à parte:

| Exp. | RMS phi / theta | Offset phi / theta (sem calibrar) | Offset phi / theta (calibrado) |
|---|---|---|---|
| C (hover) | 0,11 / 0,11 graus | +2,15 / +1,03 | +0,40 / +0,57 |
| D (passos de pitch) | 0,24 / 0,55 graus | +1,32 / +0,73 | -0,24 / +0,71 |
| E (missão) | 0,40 / 0,37 graus | +2,25 / +0,30 | -0,31 / +0,28 |

Para comparação, o inclinómetro cru (sem filtro passa-baixo) em D dá 0,89 / 1,61 graus.
Esta linha de base é favorável ao filtro, porque o inclinómetro não foi filtrado.

### 5.3 Bias artificial nos dados reais

O bias real dos gyros é ~0 (média em repouso de [-0,03, -0,02, 0,01] deg/s), por isso não há nada a convergir.
Como sugere o guia (4.4), injetou-se um bias artificial de [3, -2, 1] deg/s nos gyros da experiência D.
O bias estimado no fim foi [3,10, -2,01, 2,85] deg/s: converge bem em x e y, e pior em z.

## 6. O desvio constante de phi

Nos dados reais o filtro fica 1 a 2 graus acima do drone em phi.
A explicação é o bias do acelerómetro:

- A simulação mostra o mecanismo: um bias de acelerómetro igual ao medido em repouso (A) dá exatamente o offset esperado (+0,98 graus em phi).
- O filtro não o pode estimar: com uma única medida vetorial, o bias do acelerómetro é indistinguível de uma inclinação.
- O bias varia de uma experiência para outra. No chão, no início de C, D e E, o `ay` do acelerómetro é -0,29, -0,26 e -0,45 m/s^2 (inclinação aparente de 1,8, 1,6 e 2,6 graus), enquanto o drone marca ~0 graus. Em repouso na experiência A era -0,17 m/s^2 (1,0 grau). Calibrar com A removia só cerca de 1 grau.

Por isso implementou-se uma calibração opcional (`calibrate`, desligada por defeito): média causal de `ax` e `ay` durante os primeiros 1,5 s de cada ficheiro (o drone está no chão), congelada depois.
A descolagem só ocorre a partir de 2,2 s nas três experiências.
Só se corrigem `ax` e `ay`, porque `az` entra na inclinação só em segunda ordem.

Pressupostos e cuidados (relevantes para a discussão):

- Assume que o drone começa nivelado. Se não estiver, a calibração desloca a inclinação.
- O offset que desaparece é contra o zero do drone (o seu trim), que não é a verdade. A calibração melhora o alinhamento com o estimador interno, mas não prova que o ângulo absoluto esteja certo.
- Não usa a referência: só o acelerómetro dos primeiros 1,5 s. O único parâmetro é a duração, e não foi ajustado ao resultado.
- Não altera o RMS, nem o erro nas manobras fortes (aí o acelerómetro lê aceleração linear como inclinação).
- Recomenda-se apresentar os resultados com e sem calibração.

## 7. Limitações e o que não foi verificado

- O estimador interno do drone não é verdade. Só a simulação mede erro absoluto, e usa sensores simulados.
- A afinação foi feita nos dados de avaliação (secção 4).
- Em theta sobram 0,3 a 0,7 graus de offset em C e D que a calibração não explica. As causas possíveis (trim do drone, efeitos em voo) não foram testadas.
- O bias do gyro z só é fiável se o drone rodar. Em E estima -1,9 deg/s, quando o bias real é ~0,2 deg/s.
- `ode1` foi escolhido por ser o solver do modelo original e porque o Replay descodifica bytes (passos intermédios interpolá-los-iam). Não se comparou com RK4.
- O acelerómetro mede aceleração linear além da gravidade, o que domina o erro nas manobras fortes.

## 8. Como reproduzir

No MATLAB, em `Filter/`:

```matlab
out = run_batista_kf('E');                              % dados reais, 2 scopes
out = run_batista_kf('E', calibrate=1.5);               % com calibração do acelerómetro
out = run_batista_kf('D', bias=[3 -2 1]);               % bias artificial nos gyros
out = run_batista_kf_sim;                               % verdade conhecida, 3 scopes
out = run_batista_kf_sim(accbias=[-0.033 -0.166 0]);    % com bias de acelerómetro
```

Opções comuns: `pacing=0` (o mais depressa possível) e `scopes=false` (sem janelas).
Cada execução imprime o RMS, o offset e o bias estimado.
