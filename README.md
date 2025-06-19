#S.B.R - Self Balance Robot

**Robô autoequilibrante com controle PID**

---

##  Descrição

Este projeto tem como objetivo o desenvolvimento de um robô autoequilibrante , que utiliza um controle PID para se manter estável sobre duas rodas. O sistema utiliza dados de um sensor giroscópio para calcular os ajustes de equilíbrio em tempo real.

Uma interface web interativa será servida diretamente pelo robô, permitindo visualizar os dados de controle (como ângulo, correção do PID e status do motor) em gráficos dinâmicos. Além disso, será possível exportar os dados de uso e parâmetros PID para um banco de dados em nuvem para posterior análise.

O projeto será desenvolvido em etapas, com funcionalidades adicionais (como controle remoto e redes neurais) sendo implementadas após a conclusão do núcleo principal.

---

## 🔧 Tecnologias utilizadas

- **ESP32 WROOM-32**
- **Motor de passo NEMA17 + DRV8825**
- **MPU6050 (giroscópio + acelerômetro)**
- **SPIFFS (para hospedar a interface web)**
- **WebSocket / AJAX (para comunicação em tempo real)**
- **Supabase (como banco de dados remoto)**
- **JavaScript (Chart.js para gráficos)**
- **PlatformIO + C++**

---

##  Estrutura do projeto
```
balance-bot/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── hardware/
│   ├── schematics/
│   ├── 3d-models/
│   ├── images/
│   └── components_list.txt
│
├── firmware/
│   ├── robot/
│   └── remote/
│
├── software/
│   └── data-analyzer/
│       └── (e.g., plot_csv.py or React app)
│
├── data/
│   ├── examples/
│   └── training/
│
├── docs/
│   ├── project_description.md
│   └── references.md
│
└── tests/
├── basic_pid_test/
├── mpu_test/
└── simple_motor_control/
```
---

## Roadmap do Projeto

  - [ ] Definir escopo geral do projeto
  - [ ] **1. Montagem do robô autoequilibrante**
    - [ ] Montagem da estrutura física
    - [ ] Instalação dos motores e sensores
    - [ ] Implementação do controle PID básico
  - [ ] **2. Desenvolvimento da interface SPIFFS**
    - [ ] Criar página HTML com gráficos em tempo real
    - [ ] Comunicação com ESP32 via WebSocket ou AJAX
  - [ ] **3. Salvamento de dados no banco de dados**
    - [ ] Gerar log de uso em tempo real
    - [ ] Exportar para Supabase via dashboard ou ESP32
  - [ ] **4. Controle remoto (opcional)**
    - [ ] Implementar controle com ESP-NOW
    - [ ] Adicionar botões para movimentação manual
  - [ ] **5. Interface externa para análise (opcional)**
    - [ ] WebApp ou App desktop para explorar os dados salvos
  - [ ] **6. Aprendizado de máquina (opcional)**
    - [ ] Treinar uma rede neural com dados históricos
    - [ ] Aplicar controle adaptativo baseado em IA
