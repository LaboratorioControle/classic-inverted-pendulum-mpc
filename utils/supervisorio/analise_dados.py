"""
Script de Exemplo - Análise de Dados do ESP32
Carrega arquivo CSV e gera gráficos e estatísticas
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def analisar_dados(arquivo_csv):
    """
    Analisa dados exportados da interface ESP32
    
    Args:
        arquivo_csv: Caminho para o arquivo CSV
    """
    
    # Verificar se arquivo existe
    if not Path(arquivo_csv).exists():
        print(f"Erro: Arquivo '{arquivo_csv}' não encontrado!")
        return
    
    # Carregar dados
    print(f"Carregando dados de: {arquivo_csv}")
    df = pd.read_csv(arquivo_csv)
    
    # Converter tempo para segundos
    df['t_s'] = df['t_ms'] / 1000.0
    
    # Estatísticas básicas
    print("\n" + "="*60)
    print("ESTATÍSTICAS DOS DADOS")
    print("="*60)
    print(f"\nNúmero de amostras: {len(df)}")
    print(f"Duração total: {df['t_s'].iloc[-1] - df['t_s'].iloc[0]:.2f} segundos")
    print(f"Taxa média de amostragem: {len(df) / (df['t_s'].iloc[-1] - df['t_s'].iloc[0]):.1f} Hz")
    
    print("\n--- Ângulo θ ---")
    print(f"Média: {df['theta_deg'].mean():.2f}°")
    print(f"Desvio padrão: {df['theta_deg'].std():.2f}°")
    print(f"Mínimo: {df['theta_deg'].min():.2f}°")
    print(f"Máximo: {df['theta_deg'].max():.2f}°")
    
    print("\n--- Velocidade Angular θ̇ ---")
    print(f"Média: {df['theta_dot'].mean():.2f} rad/s")
    print(f"Desvio padrão: {df['theta_dot'].std():.2f} rad/s")
    print(f"Mínimo: {df['theta_dot'].min():.2f} rad/s")
    print(f"Máximo: {df['theta_dot'].max():.2f} rad/s")
    
    print("\n--- Posição x ---")
    print(f"Média: {df['x_cm'].mean():.2f} cm")
    print(f"Desvio padrão: {df['x_cm'].std():.2f} cm")
    print(f"Mínimo: {df['x_cm'].min():.2f} cm")
    print(f"Máximo: {df['x_cm'].max():.2f} cm")
    
    print("\n--- Velocidade Linear ẋ ---")
    print(f"Média: {df['x_dot_cm'].mean():.2f} cm/s")
    print(f"Desvio padrão: {df['x_dot_cm'].std():.2f} cm/s")
    print(f"Mínimo: {df['x_dot_cm'].min():.2f} cm/s")
    print(f"Máximo: {df['x_dot_cm'].max():.2f} cm/s")
    
    print("\n--- Sinal de Controle u ---")
    print(f"Média: {df['u'].mean():.2f}")
    print(f"Desvio padrão: {df['u'].std():.2f}")
    print(f"Mínimo: {df['u'].min():.2f}")
    print(f"Máximo: {df['u'].max():.2f}")
    
    # Criar gráficos
    criar_graficos(df)
    
    # Análise de FFT (opcional)
    # analisar_frequencias(df)


def criar_graficos(df):
    """Cria gráficos dos dados"""
    
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(16, 10))
    
    # Gráfico 1: Ângulo θ
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(df['t_s'], df['theta_deg'], 'b-', linewidth=1)
    ax1.set_ylabel('θ (°)', fontsize=11)
    ax1.set_title('Ângulo do Pêndulo', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Gráfico 2: Posição x
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(df['t_s'], df['x_cm'], 'r-', linewidth=1)
    ax2.set_ylabel('x (cm)', fontsize=11)
    ax2.set_title('Posição do Carrinho', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Gráfico 3: Sinal de Controle u
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(df['t_s'], df['u'], 'orange', linewidth=1)
    ax3.set_ylabel('u', fontsize=11)
    ax3.set_title('Sinal de Controle', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Gráfico 4: Velocidade Angular θ̇
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(df['t_s'], df['theta_dot'], 'g-', linewidth=1)
    ax4.set_ylabel('θ̇ (rad/s)', fontsize=11)
    ax4.set_title('Velocidade Angular', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Gráfico 5: Velocidade Linear ẋ
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(df['t_s'], df['x_dot_cm'], 'm-', linewidth=1)
    ax5.set_xlabel('Tempo (s)', fontsize=11)
    ax5.set_ylabel('ẋ (cm/s)', fontsize=11)
    ax5.set_title('Velocidade Linear', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Gráfico 6: Histograma u
    ax6 = plt.subplot(3, 3, 6)
    ax6.hist(df['u'], bins=50, edgecolor='black', alpha=0.7, color='orange')
    ax6.set_xlabel('u', fontsize=11)
    ax6.set_ylabel('Frequência', fontsize=11)
    ax6.set_title('Distribuição do Sinal de Controle', fontsize=12, fontweight='bold')
    ax6.grid(True, alpha=0.3, axis='y')
    
    # Gráfico 7: Plano de fase (θ vs θ̇)
    ax7 = plt.subplot(3, 3, 7)
    scatter = ax7.scatter(df['theta_deg'], df['theta_dot'], c=df['t_s'], 
                          cmap='viridis', s=1, alpha=0.5)
    ax7.set_xlabel('θ (°)', fontsize=11)
    ax7.set_ylabel('θ̇ (rad/s)', fontsize=11)
    ax7.set_title('Plano de Fase (θ, θ̇)', fontsize=12, fontweight='bold')
    ax7.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax7, label='Tempo (s)')
    
    # Gráfico 8: Plano de fase (x vs ẋ)
    ax8 = plt.subplot(3, 3, 8)
    scatter2 = ax8.scatter(df['x_cm'], df['x_dot_cm'], c=df['t_s'], 
                           cmap='plasma', s=1, alpha=0.5)
    ax8.set_xlabel('x (cm)', fontsize=11)
    ax8.set_ylabel('ẋ (cm/s)', fontsize=11)
    ax8.set_title('Plano de Fase (x, ẋ)', fontsize=12, fontweight='bold')
    ax8.grid(True, alpha=0.3)
    plt.colorbar(scatter2, ax=ax8, label='Tempo (s)')
    
    # Gráfico 9: Histograma θ
    ax9 = plt.subplot(3, 3, 9)
    ax9.hist(df['theta_deg'], bins=50, edgecolor='black', alpha=0.7)
    ax9.set_xlabel('θ (°)', fontsize=11)
    ax9.set_ylabel('Frequência', fontsize=11)
    ax9.set_title('Distribuição do Ângulo', fontsize=12, fontweight='bold')
    ax9.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.show()


def analisar_frequencias(df):
    """Análise de frequências (FFT) - Útil para sinais periódicos"""
    
    # Taxa de amostragem
    dt = np.mean(np.diff(df['t_s']))
    fs = 1.0 / dt  # Hz
    
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    
    sinais = [
        ('theta_deg', 'Ângulo θ (°)', axes[0, 0]),
        ('theta_dot', 'Velocidade Angular θ̇ (rad/s)', axes[0, 1]),
        ('x_cm', 'Posição x (cm)', axes[1, 0]),
        ('x_dot_cm', 'Velocidade ẋ (cm/s)', axes[1, 1]),
        ('u', 'Sinal de Controle u', axes[2, 0])
    ]
    
    for nome_sinal, titulo, ax in sinais:
        sinal = df[nome_sinal].values
        
        # Remover média (componente DC)
        sinal = sinal - np.mean(sinal)
        
        # FFT
        N = len(sinal)
        fft_vals = np.fft.fft(sinal)
        fft_freq = np.fft.fftfreq(N, dt)
        
        # Apenas frequências positivas
        idx_pos = fft_freq > 0
        fft_freq = fft_freq[idx_pos]
        fft_magnitude = np.abs(fft_vals[idx_pos]) / N
        
        # Plotar
        ax.plot(fft_freq, fft_magnitude, 'b-', linewidth=1)
        ax.set_xlabel('Frequência (Hz)', fontsize=11)
        ax.set_ylabel('Magnitude', fontsize=11)
        ax.set_title(f'Espectro de Frequências - {titulo}', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, min(10, fs/2))  # Limita a 10 Hz ou Nyquist
        
        # Encontrar pico de frequência
        idx_pico = np.argmax(fft_magnitude)
        freq_pico = fft_freq[idx_pico]
        ax.axvline(x=freq_pico, color='r', linestyle='--', alpha=0.7)
        ax.text(freq_pico, max(fft_magnitude)*0.9, 
                f'{freq_pico:.2f} Hz', ha='center', fontsize=9)
    
    plt.tight_layout()
    plt.show()


def calcular_metricas_controle(df):
    """Calcula métricas de desempenho do controlador"""
    
    print("\n" + "="*60)
    print("MÉTRICAS DE DESEMPENHO DO CONTROLE")
    print("="*60)
    
    # Tempo de estabilização (critério: dentro de ±2° por 1 segundo)
    theta = df['theta_deg'].values
    t = df['t_s'].values
    
    # Overshoot máximo
    overshoot = max(abs(theta))
    print(f"\nOvershoot máximo: {overshoot:.2f}°")
    
    # RMS (Root Mean Square)
    rms_theta = np.sqrt(np.mean(theta**2))
    print(f"RMS do ângulo: {rms_theta:.2f}°")
    
    # Integral do erro absoluto
    iae = np.trapz(np.abs(theta), t)
    print(f"IAE (Integral Absolute Error): {iae:.2f}")
    
    # Integral do erro quadrático
    ise = np.trapz(theta**2, t)
    print(f"ISE (Integral Square Error): {ise:.2f}")
    
    # Variação da posição do carrinho
    x_range = df['x_cm'].max() - df['x_cm'].min()
    print(f"\nVariação da posição do carrinho: {x_range:.2f} cm")


if __name__ == '__main__':
    # Exemplo de uso
    import sys
    
    if len(sys.argv) > 1:
        arquivo = sys.argv[1]
    else:
        # Se não for passado arquivo, tenta encontrar o mais recente
        import glob
        arquivos = glob.glob('dados_esp32_*.csv')
        if arquivos:
            arquivo = max(arquivos, key=lambda x: Path(x).stat().st_mtime)
            print(f"Usando arquivo mais recente: {arquivo}")
        else:
            print("Uso: python analisar_dados.py <arquivo.csv>")
            print("\nOu coloque arquivos CSV no formato 'dados_esp32_*.csv' na pasta atual")
            sys.exit(1)
    
    analisar_dados(arquivo)
    
    # Calcular métricas adicionais
    df = pd.read_csv(arquivo)
    df['t_s'] = df['t_ms'] / 1000.0
    calcular_metricas_controle(df)
    
    print("\nPressione Enter para sair...")
    input()