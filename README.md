# Projeto e Implementação Experimental de Controle Preditivo Baseado em Modelo com Estratégias de Parametrização para o Pêndulo Invertido

Este repositório contém o código-fonte e os arquivos de simulação desenvolvidos para o Trabalho de Conclusão de Curso (TCC) focado no controle de um **Pêndulo Invertido Clássico**. O projeto aborda desde a modelagem matemática e identificação de parâmetros até a implementação experimental de um **Controlador Preditivo Baseado em Modelo (MPC)** embarcado em um ESP32, utilizando estratégias de parametrização para otimização do desempenho computacional.

---

## 🏗️ Estrutura do Projeto

O projeto está dividido em duas frentes principais que se integram para permitir a validação teórica e a execução experimental:

### 1. Ambiente MATLAB (Simulação e Análise)
Localizado na raiz e nas pastas `analysis/`, `controllers/`, `models/`, `params/` e `simulation/`.
- **Modelagem:** Modelos contínuos e discretos (RK4) do sistema.
- **Identificação:** Scripts para estimativa de parâmetros físicos (atrito, inércia, constantes do motor) baseados em dados experimentais.
- **Simulação de Controle:** Implementações de controladores **LQR**, **Swing-up (Energia)** e **MPC** (com suporte a parametrização linear e restrições via qpOASES).

### 2. Ambiente ESP32 (Firmware Embarcado)
Localizado na pasta `ESP32/MPC_Inverted_Pendulum/`.
- **Controle em Tempo Real:** Implementação de uma biblioteca de controle híbrido em C++.
- **MPC Embarcado:** Uso da biblioteca `qpOASES` para resolver o problema de programação quadrática (QP) diretamente no microcontrolador.
- **Estratégias de Parametrização:** Suporte para formulações de MPC Clássico, Linear e Exponencial, visando reduzir o esforço computacional.
- **Periféricos:** Leitura de encoders, controle de motor via PWM, e interface com display OLED.

---

## 🚀 Funcionalidades Principais

| Funcionalidade | Descrição |
| :--- | :--- |
| **Controle Híbrido** | Chaveamento automático entre controle de **Swing-up** (para levar o pêndulo ao topo) e controle de **Estabilização** (MPC/LQR). |
| **MPC Parametrizado** | Implementação de técnicas para reduzir o número de variáveis de decisão, permitindo horizontes de predição maiores no ESP32. |
| **Telemetria Serial** | Protocolo de comunicação binário de alta velocidade para monitoramento de estados e sinais de controle. |
| **Supervisório Python** | Interface adicional em PyQt5 para visualização em tempo real e gravação de ensaios em CSV. |
| **Validação de Modelo** | Scripts que comparam o comportamento real do hardware com as simulações matemáticas (Métricas R²). |

---

## 🛠️ Tecnologias Utilizadas

- **Hardware:** ESP32 (Microcontrolador), Motor DC com Encoder, Encoder, Ponte H.
- **Software/Linguagens:**
  - **MATLAB/Simulink:** Projeto de controle, identificação de sistemas e simulação.
  - **C++ (PlatformIO):** Firmware embarcado de alto desempenho.
  - **Python (PyQt5):** Interface de supervisão e análise de dados.
  - **qpOASES:** Solver de programação quadrática para o MPC.

---

## 📖 Como Utilizar

### Simulação no MATLAB
1. Dê um clique duplo sobre o arquivo `classic-inverted-pendulum-mpc`. Ele irá definir o _path_ e rodar os scripts iniciais automaticamente.

### Firmware ESP32
1. Abra a pasta `ESP32/MPC_Inverted_Pendulum` no **VS Code** com a extensão **PlatformIO**.
2. Conecte o ESP32 e realize o *Upload*. O firmware já inclui as matrizes do modelo discretizado para um período de amostragem de 10ms.
3. Utilize a interface serial (115200 baud) ou o supervisório Python (`utils/supervisorio/supervisorio_esp.py`) para interagir com a bancada.

---

## 📝 Autor
Projeto desenvolvido como parte dos requisitos para obtenção do título de Bacharel em Engenharia de Controle e Automação.

**Tema:** Projeto e Implementação Experimental de Controle Preditivo Baseado em Modelo com Estratégias de Parametrização para o Pêndulo Invertido.

**Autor:** Mateus Henrique Teixeira
