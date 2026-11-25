# Contexto para o agente de simulação: modelo 3D do S.B.R mk2

Você vai montar a simulação (MuJoCo / Isaac Lab) do **S.B.R mk2**, um robô autoequilibrado de duas rodas (pêndulo invertido sobre rodas). Este texto resume o modelo 3D do Fusion 360, as malhas exportadas e os parâmetros físicos conhecidos.

Decida você o que usar: links, juntas, colisões, inércias e frames. Onde há valores do CAD e valores calibrados, **a física deve usar os calibrados/medidos**. A massa do CAD é só referência.

---

## 1. Arquivos disponíveis (`D:\isaac\sbr_meshes\`)

| Arquivo | Conteúdo | Triângulos |
|---|---|---|
| `body.stl` | Corpo inteiro: paredes, prateleiras, carcaças dos motores, parafusos, eletrônica (MPU6050), adaptador e suporte da bateria, chave liga/desliga e capa | 38.282 |
| `wheel_left.stl` | Roda esquerda (pneu e cubo, peça única em PETG) **+ eixo de aço do motor esquerdo** | 8.142 |
| `wheel_right.stl` | Roda direita **+ eixo do motor direito** | 8.142 |
| `battery.stl` | Bateria Makita CXT 12 V, na posição real de montagem | 6.484 |
| `info.json` | Centro e direção do eixo das rodas, dimensões das rodas, caixas envolventes, massa e CoM do Fusion por arquivo e do conjunto, lista dos corpos do Fusion em cada STL, itens excluídos | — |

- **Coordenadas e unidade:**
  - Todos os STL estão em **milímetros**, no **mesmo sistema de coordenadas da montagem**, sem transformação: abertos juntos, eles se encaixam.
  - São STL binários, gerados direto das malhas do Fusion.
  - Para MuJoCo, use `scale="0.001 0.001 0.001"`.
- **Malhas e colisão:**
  - As malhas **não são fechadas como um sólido único**. O `body.stl` junta 75 corpos, com sobreposições e peças internas.
  - Para colisão, prefira formas primitivas:
    - rodas como cilindros de Ø75 × 18 mm;
    - corpo como caixas ou um casco convexo.
  - Use os STL como visual.

## 2. Sistema de coordenadas (da montagem)

- **Origem:** a origem da montagem do Fusion. O eixo das rodas passa por y ≈ 0, z = 0. Em X, a origem **não** é o centro do robô (o centro fica em x = −50,05 mm).
- **+Z = para cima.** O chão (base das rodas) está em **z = −37,5 mm**.
- **X = ao longo do eixo das rodas.** **+X = esquerda do robô** (roda esquerda em x 16…34 mm).
- **−Y = frente adotada:** é o lado para onde apontam a chave liga/desliga e o USB da ESP32.
  - O robô é praticamente simétrico em Y, então a frente é convenção.
  - Se o código de controle usar a outra frente, inverta o sinal de Y e troque esquerda/direita.
- **Pitch** (o ângulo que o controle equilibra) é a rotação em torno de X.
- **Centro do robô em X:** −50,05 mm (meio entre as rodas).

## 3. Eixo e rodas (valores do `info.json`)

- **Centro do eixo (meio entre as rodas):** (−50,05; −0,26; 0,0) mm.
- **Direção do eixo:**
  - nominal: [1, 0, 0];
  - medida no CAD: [0,999994; −0,0035; 0]. Os eixos dos dois motores estão desalinhados 0,525 mm em Y no CAD (esquerdo y = −0,525, direito y = 0). Na prática, use [1, 0, 0].
- **Centro de cada roda:**
  - esquerda (25,0; −0,525; 0) mm;
  - direita (−125,1; 0; 0) mm.
- **Distância entre centros das rodas (track):** 150,1 mm.
- **Medidas da roda:**
  - Ø 75 mm externo (raio 37,5 mm);
  - 36 sulcos retos de 1,5 mm (Ø 72 no fundo);
  - largura 18 mm;
  - PETG.
- **Fixação:** a roda prende no eixo em "D" de Ø5 mm por parafuso M3 lateral, sem folga prevista.

**Atenção:** o STL de cada roda inclui o **eixo de aço do motor** (componente `Cuerpo003`, ~57 g, 62 mm de comprimento, Ø5 com um colar de Ø15).
- Ele gira com a roda, por isso foi para o arquivo da roda e não para o corpo.
- Consequências:
  - a caixa envolvente da roda entra ~44 mm para dentro do corpo;
  - a massa da roda no Fusion (91 g) inclui o eixo. Só pneu e cubo: 34,4 g sólido.
- O rotor do motor não está modelado à parte.

## 4. O que é o robô (peças e alturas)

Estrutura impressa em **PLA**, de baixo para cima (z em mm, acima do eixo das rodas):

| Item | Componente(s) Fusion | z (mm) | Observação |
|---|---|---|---|
| Chão | — | −37,5 | |
| 2 motores NEMA 17 (39 mm), eixo em z=0 | `NEMA 17 Stepper Motor 39mm:1/:2` | −21 a +21 (conector até 26) | Motor 1 = esquerdo (+X), motor 2 = direito |
| 2 paredes laterais (esq/dir) | `CHASSI_SideFrame_LEFT_mk2`, `..._RIGHT_mk2` | −21,2 a 122,8 | Abas a ±22–25 mm em Y; prendem motores e prateleiras |
| Prateleira do IMU | `CHASSI_IMU_SHELF` | 27 a 39,5 | 120 × 42 × 8 mm |
| IMU GY-521 (MPU6050) | `MPU_6050` | chip em ~37 | Ver seção 6 |
| Prateleira de eletrônica (PCB, ESP32, drivers) | `CHASSI_Pcb_SHELF` | 42 a 52 | A placa de controle ainda não está no CAD; altura útil ~25 mm |
| Bateria (pendurada por baixo da prateleira) | `battery makita 12v.SLDPRT` | 77 a 121 | CoG ~94 mm |
| Adaptador Makita e ligação | `makita_10.8v_v38`, `LINK-SHELF-to-B_ADPTOR` | 105,5 a 127 | No `body.stl` |
| Prateleira da bateria | `CHASSI_Battery_SHELF` | 98 a 127 | |
| Suporte da chave, chave de 21 mm, capa | `CHASSI_PowerSwitch_HOLDER`, `21mm round on-off switch`, `CHASSI_PowerSwitch_COVER` | 125,5 a 150 | Lado −Y |
| Rodas | `RODA_mk2:1` (esq), `RODA_mk2:2` (dir) | −37,5 a 37,5 | |

**Dimensões gerais:**
- altura total do chão ao topo da chave: ~187,5 mm;
- largura de roda a roda por fora: ~168 mm (x −134,1 a 34,0);
- profundidade: ~81 mm, limitada pela bateria (y −40 a +41);
- distância entre as paredes, por dentro: 120 mm.

## 5. Massa e inércia

**Do CAD** (Fusion, materiais do modelo, peças impressas 100% sólidas, NEMA 17 a 425 g). São só referência:

| Parte | Massa | CoM (mm) |
|---|---|---|
| body.stl | 1,197 kg | (−50,03; −2,24; 35,49) |
| wheel_left.stl (com eixo) | 0,091 kg | (4,14; −0,48; 0,0) |
| wheel_right.stl (com eixo) | 0,091 kg | (−104,32; −0,05; 0,0) |
| battery.stl | 0,210 kg | (−50,00; 1,77; 93,91) |
| Conjunto | 1,589 kg | (−50,03; −1,48; 39,15) |

**Calibrados para o robô real.** O mk1 pesou 1,00 kg na balança. A calibração usa:
- motor de 280 g;
- impressas a 70% da massa sólida;
- chave de 10 g;
- eletrônica de 52 g em z ≈ 65 mm.

Com isso o mk1 calculado dá 1,004 kg. **Use estes valores:**

```yaml
mass_total_kg: 1.068              # mk2 previsto (mk1 medido = 1.00 kg)
mass_body_kg: 1.019               # tudo menos as rodas (parte que pendula)
com_body_height_above_axle_m: 0.0373
com_total_height_above_axle_m: 0.0357
com_offset_y_m: -0.0002           # ~0
body_pitch_inertia_about_axle_kgm2: 3.93e-3
body_pitch_inertia_about_com_kgm2: 2.52e-3
yaw_inertia_total_kgm2: 1.98e-3
yaw_inertia_body_kgm2: 1.68e-3
wheel_radius_m: 0.0375
wheel_width_m: 0.018
wheel_mass_kg: 0.0241             # cada, só pneu+cubo (34.4 g sólido x 0.7)
wheel_inertia_about_axle_kgm2: 2.16e-5   # sem rotor do motor
motor_rotor_inertia_kgm2: ~5.4e-6        # típico NEMA 17, somar à roda se modelar o rotor
track_width_m: 0.1501
axle_height_above_ground_m: 0.0375
```

- **Precisão:** massa ±5%; CoG e inércias ±15%. A incerteza vem da divisão de massa entre motores, impressas e eletrônica.
- Pesar o robô mk2 montado vai melhorar esses números.

## 6. IMU

- **Sensor:** GY-521 (MPU6050), com a placa horizontal e os componentes para cima.
- **Posição do chip:** [0,6; −0,2; 37,0] mm **relativa ao centro do eixo**. Nas coordenadas da montagem, ≈ (−49,5; −0,5; 37).
- **Orientação:**
  - o lado de 21 mm da placa (a fileira de pinos) fica paralelo a X;
  - o header de 90° fica para −Y;
  - os furos de fixação ficam em +Y.
- O eixo do giroscópio que mede o pitch deve ser o paralelo a X. Confirme o sinal no hardware.
- No mk1 o IMU ficava a ~131 mm do eixo. No mk2 fica a 37 mm, e a aceleração tangencial medida cai ~3,5×. Filtros e ganhos ajustados no mk1 devem ser revistos.

## 7. Atuação e eletrônica (para o modelo de atuador)

- **Motores:** 2× NEMA 17 de passo (1,8° = 200 passos/volta), cada um acionando uma roda diretamente, sem redução.
- **Drivers:** soquetes Pololu/StepStick que aceitam A4988, DRV8825 ou TMC2209.
  - O micropasso é escolhido por jumper: 1/16 típico; TMC2209 com UART opcional.
  - O controle é por STEP/DIR (velocidade e aceleração das rodas), não por torque.
  - Pinos da ESP32: STEP/DIR esquerdo 32/33, direito 25/26, EN 27.
- **Controle:** ESP32, com IMU no I2C (21/22) e interrupção no GPIO39.
- **Bateria:** Makita CXT 12 V máx., 1,5 Ah (3S Li-ion, 10,8 V nominal, 12,6 V cheia, ~9 V vazia).
- **Placa de controle:** 104 × 48 mm na prateleira de eletrônica. Está projetada no KiCad, mas **não existe no CAD do Fusion**. A massa dela entra na calibração como "eletrônica 52 g em z ≈ 65 mm".

## 8. O que ficou de fora das malhas

- **Ocultos no modelo, peças antigas ou alternativas:**
  - paredes do mk1;
  - rodas antigas (`Whell Medium`, `Wheel Ready`, `WheelSmall`, `Whell`, `Fast Wheel Couple`);
  - suporte aranha do IMU (`CHASSI_mpu_HOLDER`);
  - bloco antigo da bateria (`Battery :1`).
- **Peças pequenas ocultas:** terminais e tampa do adaptador Makita.
- **Suporte de bancada** (`SUPORTE_Vertical_mini` e `_v1_macico`): não faz parte do robô. Ele levanta o robô 8 mm do chão para testes parados.
- **Não modelados:**
  - parafuso e porca de fixação das rodas;
  - cabos;
  - placa de controle;
  - rotor do motor.

## 9. Decisões que ficam com você

1. Se o eixo do motor deve ficar na malha da roda (como está), ser removido, ou ter a inércia somada à roda. O rotor equivalente é ~5,4e-6 kg·m².
2. As formas de colisão: primitivas para rodas e corpo (recomendado) ou casco convexo da malha.
3. Qual frente usar (−Y adotado aqui) e como mapear esquerda/direita no código.
4. Se a bateria fica como link separado fixo no corpo (útil para variar massa e CoG) ou é fundida ao corpo.
5. O modelo do atuador: velocidade comandada via STEP/DIR, com limite de aceleração e torque do NEMA 17 (~0,4 N·m de holding típico, caindo com a rotação), ou torque ideal.
6. O atrito e o contato roda-chão. O pneu é PETG impresso com sulcos, sem borracha: atrito moderado.
