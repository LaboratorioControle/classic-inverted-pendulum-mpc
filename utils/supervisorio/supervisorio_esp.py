"""
Interface Gráfica para Comunicação com ESP32
Autor: Sistema de Controle
Data: 2026
"""

import sys
import struct
import time
from datetime import datetime
from collections import deque
import csv

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QLineEdit, 
                             QComboBox, QGroupBox, QGridLayout, QTextEdit,
                             QFileDialog, QCheckBox, QSpinBox, QDoubleSpinBox,
                             QTabWidget, QSplitter, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import serial
import serial.tools.list_ports


class LogData:
    """Estrutura de dados recebida do ESP32"""
    def __init__(self, t_ms=0, theta_deg=0.0, theta_dot=0.0, x_cm=0.0, x_dot_cm=0.0, u=0.0, yref=0.0, tempo_computacional=0.0, cod_erro=0.0):
        self.t_ms = t_ms
        self.theta_deg = theta_deg
        self.theta_dot = theta_dot
        self.x_cm = x_cm
        self.x_dot_cm = x_dot_cm
        self.u = u
        self.yref = yref
        self.tempo_computacional = tempo_computacional
        self.cod_erro = cod_erro
    
    @staticmethod
    def from_bytes(data):
        """Converte bytes recebidos para LogData"""
        # Formato: uint32_t (I) + 8x float (f)
        unpacked = struct.unpack('<I8f', data)
        return LogData(
            t_ms=unpacked[0],
            theta_deg=unpacked[1],
            theta_dot=unpacked[2],
            x_cm=unpacked[3],
            x_dot_cm=unpacked[4],
            u=unpacked[5],
            yref=unpacked[6],
            tempo_computacional=unpacked[7],
            cod_erro=unpacked[8]
        )
    
    def to_dict(self):
        """Converte para dicionário"""
        return {
            't_ms': self.t_ms,
            'theta_deg': self.theta_deg,
            'theta_dot': self.theta_dot,
            'x_cm': self.x_cm,
            'x_dot_cm': self.x_dot_cm,
            'u': self.u,
            'yref': self.yref,
            'tempo_computacional': self.tempo_computacional,
            'cod_erro': self.cod_erro
        }


class SerialReaderThread(QThread):
    """Thread para leitura contínua da serial"""
    data_received = pyqtSignal(LogData)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_conn = None
        
    def run(self):
        """Loop principal de leitura"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            
            buffer = bytearray()
            HEADER = bytes([0xAA, 0x55])
            DATA_SIZE = 36  # 4 bytes (uint32_t) + 32 bytes (8x float)
            
            while self.running:
                if self.serial_conn.in_waiting > 0:
                    # Lê dados disponíveis
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(chunk)
                    
                    # Procura pelo header
                    while len(buffer) >= len(HEADER) + DATA_SIZE:
                        # Procura o header no buffer
                        header_idx = buffer.find(HEADER)
                        
                        if header_idx == -1:
                            # Header não encontrado, limpa buffer antigo
                            buffer = buffer[-(len(HEADER) + DATA_SIZE):]
                            break
                        
                        if header_idx > 0:
                            # Remove dados antes do header
                            buffer = buffer[header_idx:]
                        
                        # Verifica se temos dados suficientes após o header
                        if len(buffer) >= len(HEADER) + DATA_SIZE:
                            # Extrai os dados
                            data_bytes = buffer[len(HEADER):len(HEADER) + DATA_SIZE]
                            
                            try:
                                log_data = LogData.from_bytes(data_bytes)
                                self.data_received.emit(log_data)
                            except struct.error as e:
                                self.error_occurred.emit(f"Erro ao decodificar dados: {e}")
                            
                            # Remove dados processados do buffer
                            buffer = buffer[len(HEADER) + DATA_SIZE:]
                        else:
                            break
                else:
                    self.msleep(5)  # Pequena pausa se não há dados
                    
        except serial.SerialException as e:
            self.error_occurred.emit(f"Erro na porta serial: {e}")
        except Exception as e:
            self.error_occurred.emit(f"Erro inesperado: {e}")
        finally:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
    
    def stop(self):
        """Para a thread de leitura"""
        self.running = False
        self.wait()
    
    def send_command(self, command):
        """Envia comando para o ESP32"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write((command + '\n').encode())
                return True
            except Exception as e:
                self.error_occurred.emit(f"Erro ao enviar comando: {e}")
                return False
        return False


class MainWindow(QMainWindow):
    """Janela principal da aplicação"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interface ESP32 - Sistema de Controle")
        self.setGeometry(100, 100, 1400, 900)
        
        # Variáveis de controle
        self.serial_thread = None
        self.recording = False
        self.recorded_data = []
        self.plot_updating = True  # Controle de atualização dos gráficos
        
        # Configuração da janela de tempo para plotagem
        self.plot_time_window = 15.0  # segundos - janela deslizante de 15s
        
        # Buffers de dados para plotagem (sem limite fixo, será filtrado por tempo)
        self.time_data = deque()
        self.theta_data = deque()
        self.theta_dot_data = deque()
        self.x_data = deque()
        self.x_dot_data = deque()
        self.u_data = deque()
        self.yref_data = deque()
        
        # Tempo inicial (para plotagem relativa)
        self.time_offset = None
        
        self.init_ui()

        # Timer para atualizar gráficos (20 FPS)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(50)
        
    def init_ui(self):
        """Inicializa a interface gráfica"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # === SEÇÃO DE CONEXÃO ===
        conn_group = self.create_connection_group()
        main_layout.addWidget(conn_group)
        
        # === SPLITTER PARA CONTROLES E GRÁFICOS ===
        splitter = QSplitter(Qt.Horizontal)
        
        # Painel esquerdo - Controles
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Comandos manuais
        left_layout.addWidget(self.create_manual_commands_group())
        
        # Comando de degrau
        left_layout.addWidget(self.create_step_command_group())
        
        # Comando senoide
        left_layout.addWidget(self.create_sine_command_group())
        
        # Controlador LQR
        left_layout.addWidget(self.create_lqr_command_group())
        
        # Console de mensagens
        left_layout.addWidget(self.create_console_group())
        
        left_layout.addStretch()
        
        # Painel direito - Gráficos
        right_panel = self.create_plots_panel()
        
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        
        main_layout.addWidget(splitter)
        
        # === SEÇÃO DE GRAVAÇÃO ===
        rec_group = self.create_recording_group()
        main_layout.addWidget(rec_group)
        
    def create_connection_group(self):
        """Cria grupo de conexão serial"""
        group = QGroupBox("Conexão Serial")
        layout = QHBoxLayout()
        
        layout.addWidget(QLabel("Porta:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton("🔄 Atualizar")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        layout.addWidget(QLabel("Baudrate:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '57600', '115200', '230400', '460800'])
        self.baudrate_combo.setCurrentText('115200')
        layout.addWidget(self.baudrate_combo)
        
        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("Desconectado")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        
        group.setLayout(layout)
        return group
    
    def create_manual_commands_group(self):
        """Cria grupo de comandos manuais"""
        group = QGroupBox("Comandos Manuais")
        layout = QGridLayout()
        
        btn_left = QPushButton("← Esquerda (L)")
        btn_left.clicked.connect(lambda: self.send_command_with_stop('L'))
        layout.addWidget(btn_left, 0, 0)
        
        btn_right = QPushButton("Direita (R) →")
        btn_right.clicked.connect(lambda: self.send_command_with_stop('R'))
        layout.addWidget(btn_right, 0, 1)
        
        btn_stop = QPushButton("⏸ Parar (P)")
        btn_stop.clicked.connect(lambda: self.send_command('P'))
        layout.addWidget(btn_stop, 1, 0)
        
        btn_zero = QPushButton("🔄 Zerar Encoders (Z)")
        btn_zero.clicked.connect(lambda: self.send_command('Z'))
        layout.addWidget(btn_zero, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def create_step_command_group(self):
        """Cria grupo de comando de degrau"""
        group = QGroupBox("Comando de Degrau (D)")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Duração (ms):"), 0, 0)
        self.step_duration = QSpinBox()
        self.step_duration.setRange(0, 60000)
        self.step_duration.setValue(500)
        layout.addWidget(self.step_duration, 0, 1)
        
        layout.addWidget(QLabel("Intensidade (0-255):"), 1, 0)
        self.step_intensity = QSpinBox()
        self.step_intensity.setRange(0, 255)
        self.step_intensity.setValue(200)
        layout.addWidget(self.step_intensity, 1, 1)
        
        layout.addWidget(QLabel("Sentido:"), 2, 0)
        self.step_direction = QComboBox()
        self.step_direction.addItems(['L (Esquerda)', 'R (Direita)'])
        layout.addWidget(self.step_direction, 2, 1)
        
        btn_send_step = QPushButton("Enviar Degrau")
        btn_send_step.clicked.connect(self.send_step_command)
        layout.addWidget(btn_send_step, 3, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_sine_command_group(self):
        """Cria grupo de comando senoidal"""
        group = QGroupBox("Comando Senoide (S)")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Amplitude (0-255):"), 0, 0)
        self.sine_amplitude = QSpinBox()
        self.sine_amplitude.setRange(0, 255)
        self.sine_amplitude.setValue(100)
        layout.addWidget(self.sine_amplitude, 0, 1)
        
        layout.addWidget(QLabel("Frequência (Hz):"), 1, 0)
        self.sine_frequency = QSpinBox()
        self.sine_frequency.setRange(1, 100)
        self.sine_frequency.setSingleStep(1)
        self.sine_frequency.setValue(1)
        layout.addWidget(self.sine_frequency, 1, 1)
        
        layout.addWidget(QLabel("Duração (ms):"), 2, 0)
        self.sine_duration = QSpinBox()
        self.sine_duration.setRange(1, 10000)
        self.sine_duration.setSingleStep(1)
        self.sine_duration.setValue(5000)
        layout.addWidget(self.sine_duration, 2, 1)
        
        btn_send_sine = QPushButton("Enviar Senoide")
        btn_send_sine.clicked.connect(self.send_sine_command)
        layout.addWidget(btn_send_sine, 3, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_lqr_command_group(self):
        """Cria grupo de comando LQR"""
        group = QGroupBox("Controlador LQR")
        layout = QGridLayout()
        
        # Ganhos K
        self.lqr_k = []
        for i in range(4):
            layout.addWidget(QLabel(f"K[{i}]:"), i, 0)
            k_spin = QDoubleSpinBox()
            k_spin.setRange(-1000.0, 1000.0)
            k_spin.setSingleStep(0.1)
            k_spin.setDecimals(3)
            k_spin.setValue(0.0)
            layout.addWidget(k_spin, i, 1)
            self.lqr_k.append(k_spin)
        
        layout.addWidget(QLabel("K_swing:"), 4, 0)
        self.lqr_k_swing = QDoubleSpinBox()
        self.lqr_k_swing.setRange(-1000.0, 1000.0)
        self.lqr_k_swing.setSingleStep(0.1)
        self.lqr_k_swing.setDecimals(3)
        self.lqr_k_swing.setValue(0.0)
        layout.addWidget(self.lqr_k_swing, 4, 1)
        
        btn_activate_lqr = QPushButton("Ativar LQR (Q)")
        btn_activate_lqr.clicked.connect(self.send_lqr_command)
        layout.addWidget(btn_activate_lqr, 5, 0, 1, 2)
        
        btn_deactivate_lqr = QPushButton("Desativar LQR (X)")
        btn_deactivate_lqr.clicked.connect(lambda: self.send_command('X'))
        layout.addWidget(btn_deactivate_lqr, 6, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    def create_console_group(self):
        """Cria console de mensagens"""
        group = QGroupBox("Console")
        layout = QVBoxLayout()
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(150)
        self.console.setFont(QFont("Courier", 9))
        layout.addWidget(self.console)
        
        clear_btn = QPushButton("Limpar Console")
        clear_btn.clicked.connect(self.console.clear)
        layout.addWidget(clear_btn)
        
        group.setLayout(layout)
        return group
    
    def create_plots_panel(self):
        """Cria painel de gráficos"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Configuração do pyqtgraph
        pg.setConfigOptions(antialias=False)
        
        # Layout superior com 2 colunas
        top_layout = QHBoxLayout()
        
        # Coluna 1: Ângulo θ
        self.plot_theta = pg.PlotWidget(title="Ângulo θ (graus)")
        self.plot_theta.setLabel('left', 'θ', units='°')
        self.plot_theta.setLabel('bottom', 'Tempo', units='s')
        self.plot_theta.showGrid(x=True, y=True)
        self.curve_theta = self.plot_theta.plot(pen='y', name='θ')
        top_layout.addWidget(self.plot_theta)
        
        # Coluna 2: Posição x
        self.plot_x = pg.PlotWidget(title="Posição x (cm)")
        self.plot_x.setLabel('left', 'x', units='cm')
        self.plot_x.setLabel('bottom', 'Tempo', units='s')
        self.plot_x.showGrid(x=True, y=True)
        self.curve_x = self.plot_x.plot(pen='g', name='x')
        top_layout.addWidget(self.plot_x)
        
        layout.addLayout(top_layout)
        
        # Layout do meio com 2 colunas
        middle_layout = QHBoxLayout()
        
        # Coluna 1: Velocidade angular θ̇
        self.plot_theta_dot = pg.PlotWidget(title="Velocidade Angular θ̇")
        self.plot_theta_dot.setLabel('left', 'θ̇', units='°/s')
        self.plot_theta_dot.setLabel('bottom', 'Tempo', units='s')
        self.plot_theta_dot.showGrid(x=True, y=True)
        self.curve_theta_dot = self.plot_theta_dot.plot(pen='c', name='θ̇')
        middle_layout.addWidget(self.plot_theta_dot)
        
        # Coluna 2: Velocidade linear ẋ
        self.plot_x_dot = pg.PlotWidget(title="Velocidade Linear ẋ (cm/s)")
        self.plot_x_dot.setLabel('left', 'ẋ', units='cm/s')
        self.plot_x_dot.setLabel('bottom', 'Tempo', units='s')
        self.plot_x_dot.showGrid(x=True, y=True)
        self.curve_x_dot = self.plot_x_dot.plot(pen='m', name='ẋ')
        middle_layout.addWidget(self.plot_x_dot)
        
        layout.addLayout(middle_layout)

        bot_layout = QHBoxLayout()

        self.plot_u = pg.PlotWidget(title="Sinal de Controle u")
        self.plot_u.setLabel('left', 'u')
        self.plot_u.setLabel('bottom', 'Tempo', units='s')
        self.plot_u.showGrid(x=True, y=True)
        self.curve_u = self.plot_u.plot(pen='r', name='u')

        self.plot_yref = pg.PlotWidget(title="Set Point yref")
        self.plot_yref.setLabel('left', 'yref')
        self.plot_yref.setLabel('bottom', 'Tempo', units='s')
        self.plot_yref.showGrid(x=True, y=True)
        self.curve_yref = self.plot_yref.plot(pen='b', name='yref')

        bot_layout.addWidget(self.plot_u)
        bot_layout.addWidget(self.plot_yref)

        layout.addLayout(bot_layout)
        
        return widget
    
    def create_recording_group(self):
        """Cria grupo de gravação de dados"""
        group = QGroupBox("Gravação de Dados e Visualização")
        layout = QHBoxLayout()
        
        # Controle de plotagem
        self.plot_toggle_btn = QPushButton("⏸ Pausar Gráficos")
        self.plot_toggle_btn.clicked.connect(self.toggle_plot_update)
        self.plot_toggle_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        layout.addWidget(self.plot_toggle_btn)
        
        self.clear_plots_btn = QPushButton("🧹 Limpar Gráficos")
        self.clear_plots_btn.clicked.connect(self.clear_plots)
        layout.addWidget(self.clear_plots_btn)
        
        # Controle de janela de tempo
        layout.addWidget(QLabel("Janela:"))
        self.time_window_spin = QSpinBox()
        self.time_window_spin.setRange(5, 120)
        self.time_window_spin.setValue(15)
        self.time_window_spin.setSuffix(" s")
        self.time_window_spin.valueChanged.connect(self.update_time_window)
        layout.addWidget(self.time_window_spin)
        
        # Separador visual
        separator = QLabel("|")
        separator.setStyleSheet("color: gray; font-size: 20px;")
        layout.addWidget(separator)
        
        # Gravação
        self.record_btn = QPushButton("⏺ Iniciar Gravação")
        self.record_btn.clicked.connect(self.toggle_recording)
        layout.addWidget(self.record_btn)
        
        self.export_btn = QPushButton("💾 Exportar CSV")
        self.export_btn.clicked.connect(self.export_to_csv)
        self.export_btn.setEnabled(False)
        layout.addWidget(self.export_btn)
        
        self.record_label = QLabel("Pontos gravados: 0")
        layout.addWidget(self.record_label)
        
        self.clear_data_btn = QPushButton("🗑 Limpar Dados")
        self.clear_data_btn.clicked.connect(self.clear_recorded_data)
        layout.addWidget(self.clear_data_btn)
        
        layout.addStretch()
        
        group.setLayout(layout)
        return group
    
    def refresh_ports(self):
        """Atualiza lista de portas seriais disponíveis"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def toggle_connection(self):
        """Conecta/desconecta da porta serial"""
        if self.serial_thread is None or not self.serial_thread.isRunning():
            # Conectar
            port_text = self.port_combo.currentText()
            if not port_text:
                self.log_message("Erro: Nenhuma porta selecionada")
                return
            
            port = port_text.split(' - ')[0]
            baudrate = int(self.baudrate_combo.currentText())
            
            self.serial_thread = SerialReaderThread(port, baudrate)
            self.serial_thread.data_received.connect(self.on_data_received)
            self.serial_thread.error_occurred.connect(self.on_error)
            self.serial_thread.start()
            
            self.connect_btn.setText("Desconectar")
            self.status_label.setText("Conectado")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.log_message(f"Conectado em {port} @ {baudrate} baud")
            
            # Resetar tempo offset
            self.time_offset = None
        else:
            # Desconectar
            self.serial_thread.stop()
            self.connect_btn.setText("Conectar")
            self.status_label.setText("Desconectado")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.log_message("Desconectado")
    
    def on_data_received(self, log_data):
        """Callback quando dados são recebidos"""
        # Ajusta tempo relativo
        if self.time_offset is None:
            self.time_offset = log_data.t_ms
        
        time_s = (log_data.t_ms - self.time_offset) / 1000.0
        
        # SEMPRE adiciona aos buffers (mesmo com gráficos pausados)
        self.time_data.append(time_s)
        self.theta_data.append(log_data.theta_deg)
        self.theta_dot_data.append(log_data.theta_dot)
        self.x_data.append(log_data.x_cm)
        self.x_dot_data.append(log_data.x_dot_cm)
        self.u_data.append(log_data.u)
        self.yref_data.append(log_data.yref)
        
        # Remove dados mais antigos que a janela de tempo (mantém últimos 15s)
        if len(self.time_data) > 0:
            cutoff_time = time_s - self.plot_time_window
            
            # Remove elementos do início enquanto forem mais antigos que cutoff_time
            while len(self.time_data) > 0 and self.time_data[0] < cutoff_time:
                self.time_data.popleft()
                self.theta_data.popleft()
                self.theta_dot_data.popleft()
                self.x_data.popleft()
                self.x_dot_data.popleft()
                self.u_data.popleft()
                self.yref_data.popleft()
        
        # Grava dados se estiver gravando (independente dos gráficos)
        if self.recording:
            self.recorded_data.append(log_data.to_dict())
            self.record_label.setText(f"Pontos gravados: {len(self.recorded_data)}")
    
    def on_error(self, error_msg):
        """Callback quando ocorre erro"""
        self.log_message(f"ERRO: {error_msg}")
        QMessageBox.critical(self, "Erro", error_msg)
    
    def send_command(self, command):
        """Envia comando para o ESP32"""
        if self.serial_thread and self.serial_thread.isRunning():
            if self.serial_thread.send_command(command):
                self.log_message(f"→ Enviado: {command}")
            else:
                self.log_message(f"✗ Falha ao enviar: {command}")
        else:
            self.log_message("Erro: Não conectado")
    
    def send_command_with_stop(self, command):
        """Envia comando seguido de P após 50ms"""
        if self.serial_thread and self.serial_thread.isRunning():
            # Envia o comando inicial
            if self.serial_thread.send_command(command):
                self.log_message(f"→ Enviado: {command}")
                
                # Agenda o envio de P após 50ms
                QTimer.singleShot(50, lambda: self.send_command('P'))
            else:
                self.log_message(f"✗ Falha ao enviar: {command}")
        else:
            self.log_message("Erro: Não conectado")
    
    def send_step_command(self):
        """Envia comando de degrau"""
        duration = self.step_duration.value()
        intensity = self.step_intensity.value()
        direction = 'L' if 'Esquerda' in self.step_direction.currentText() else 'R'
        
        command = f"D,{duration},{intensity},{direction}"
        self.send_command(command)
    
    def send_sine_command(self):
        """Envia comando senoidal"""
        amplitude = self.sine_amplitude.value()
        frequency = self.sine_frequency.value()
        duration = self.sine_duration.value()
        
        command = f"S,{amplitude},{frequency},{duration}"
        self.send_command(command)
    
    def send_lqr_command(self):
        """Envia comando LQR"""
        k_values = [k.value() for k in self.lqr_k]
        k_swing = self.lqr_k_swing.value()
        
        command = f"Q,{k_values[0]},{k_values[1]},{k_values[2]},{k_values[3]},{k_swing}"
        self.send_command(command)
    
    def toggle_recording(self):
        """Inicia/para gravação de dados"""
        if not self.recording:
            self.recording = True
            self.recorded_data = []
            self.record_btn.setText("⏹ Parar Gravação")
            self.record_btn.setStyleSheet("background-color: red; color: white;")
            self.export_btn.setEnabled(False)
            self.log_message("Gravação iniciada")
        else:
            self.recording = False
            self.record_btn.setText("⏺ Iniciar Gravação")
            self.record_btn.setStyleSheet("")
            self.export_btn.setEnabled(len(self.recorded_data) > 0)
            self.log_message(f"Gravação parada - {len(self.recorded_data)} pontos")
    
    def export_to_csv(self):
        """Exporta dados gravados para CSV"""
        if not self.recorded_data:
            QMessageBox.warning(self, "Aviso", "Nenhum dado para exportar")
            return
        
        filename, _ = QFileDialog.getSaveFileName(
            self, "Exportar CSV", 
            f"dados_esp32_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )
        
        if filename:
            try:
                with open(filename, 'w', newline='') as csvfile:
                    fieldnames = ['t_ms', 'theta_deg', 'theta_dot', 'x_cm', 'x_dot_cm', 'u', 'yref', 'tempo_computacional', 'cod_erro']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    writer.writeheader()
                    writer.writerows(self.recorded_data)
                
                self.log_message(f"✓ Dados exportados: {filename}")
                QMessageBox.information(self, "Sucesso", 
                                       f"Dados exportados com sucesso!\n{len(self.recorded_data)} pontos salvos")
            except Exception as e:
                self.log_message(f"✗ Erro ao exportar: {e}")
                QMessageBox.critical(self, "Erro", f"Erro ao exportar: {e}")
    
    def clear_recorded_data(self):
        """Limpa dados gravados"""
        reply = QMessageBox.question(
            self, "Confirmar", 
            f"Deseja limpar {len(self.recorded_data)} pontos gravados?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.recorded_data = []
            self.record_label.setText("Pontos gravados: 0")
            self.export_btn.setEnabled(False)
            self.log_message("Dados gravados limpos")
    
    def clear_plots(self):
        """Limpa todos os gráficos"""
        # Limpa os buffers de dados
        self.time_data.clear()
        self.theta_data.clear()
        self.theta_dot_data.clear()
        self.x_data.clear()
        self.x_dot_data.clear()
        self.u_data.clear()
        self.yref_data.clear()
        
        # Limpa as curvas nos gráficos
        self.curve_theta.setData([], [])
        self.curve_theta_dot.setData([], [])
        self.curve_x.setData([], [])
        self.curve_x_dot.setData([], [])
        self.curve_u.setData([], [])
        self.curve_yref.setData([], [])
        
        # Reseta o offset de tempo
        self.time_offset = None
        
        self.log_message("Gráficos limpos")
    
    def toggle_plot_update(self):
        """Pausa/retoma atualização dos gráficos"""
        self.plot_updating = not self.plot_updating
        
        if self.plot_updating:
            # Retomando - atualiza com dados acumulados
            self.plot_toggle_btn.setText("⏸ Pausar Gráficos")
            self.plot_toggle_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            
            # Atualiza gráficos com todos os dados acumulados
            if len(self.time_data) > 0:
                time_array = list(self.time_data)
                self.curve_theta.setData(time_array, list(self.theta_data))
                self.curve_theta_dot.setData(time_array, list(self.theta_dot_data))
                self.curve_x.setData(time_array, list(self.x_data))
                self.curve_x_dot.setData(time_array, list(self.x_dot_data))
                self.curve_u.setData(time_array, list(self.u_data))
                self.curve_yref.setData(time_array, list(self.yref_data))
            
            self.log_message("Gráficos retomados - atualizando em tempo real")
        else:
            # Pausando
            self.plot_toggle_btn.setText("▶ Retomar Gráficos")
            self.plot_toggle_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
            self.log_message("Gráficos pausados - dados continuam sendo recebidos")
    
    def update_time_window(self, value):
        """Atualiza a janela de tempo da plotagem"""
        self.plot_time_window = float(value)
        self.log_message(f"Janela de tempo alterada para {value}s")

    def update_plots(self):

        if not self.plot_updating:
            return

        if len(self.time_data) == 0:
            return

        time_array = list(self.time_data)

        self.curve_theta.setData(time_array, list(self.theta_data))
        self.curve_theta_dot.setData(time_array, list(self.theta_dot_data))
        self.curve_x.setData(time_array, list(self.x_data))
        self.curve_x_dot.setData(time_array, list(self.x_dot_data))
        self.curve_u.setData(time_array, list(self.u_data))
        self.curve_yref.setData(time_array, list(self.yref_data))
    
    def log_message(self, message):
        """Adiciona mensagem ao console"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.console.append(f"[{timestamp}] {message}")
    
    def closeEvent(self, event):
        """Evento de fechamento da janela"""
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()