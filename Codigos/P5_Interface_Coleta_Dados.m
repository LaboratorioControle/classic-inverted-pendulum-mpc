function Interface_Coleta_Dados()

        clear; clc; close all;

    % ==============================
    % JANELA PRINCIPAL COM ABAS
    % ==============================
        f = uifigure('Name','Software - Pêndulo Invertido', 'Resize','off', 'Position',[100 100 1200 650], 'Icon','Imagens/logoPipaVovo.png');
        movegui(f,'center');
        tabgp = uitabgroup(f, 'Units', 'normalized', 'Position', [0 0.05 1 0.95]);
        
        tab1 = uitab(tabgp,'Title','Análise Malha Aberta');
        tab2 = uitab(tabgp,'Title','Controle Manual');
        tab3 = uitab(tabgp,'Title','Configurações');
        tab4 = uitab(tabgp,'Title','Controlador - LQR + Swing Up Energia');

        painelLogo = uipanel(f, ...
            'Units', 'normalized', ...
            'Position', [0 0 1 0.05], ...     % <--- painel ocupa a parte inferior
            'BackgroundColor', [0.90 0.90 0.90], ...
            'BorderType','none');
        
        uiimage(painelLogo, ...
            'ImageSource', 'Imagens/logoUfla.png', ...  % coloque o nome do seu arquivo aqui
            'Position', [1130 -15 60 60]);   % centraliza horizontalmente

        uilabel(painelLogo, ...
            'Text', 'GAT125 - SOFTWARE PÊNDULO INVERTIDO', ...
            'FontSize', 14, ...
            'FontWeight', 'bold', ...
            'Position',[10,-485,1000,1000]);
        
        % Variáveis persistentes (compartilhadas)
        dados = [];
        nome_arquivo = "";
        sManual = [];
    
    %% ==============================
    % ABA 3 - CONFIGURAÇÕES
    % ==============================
    
    % --- Painel principal para Configurações ESP32 ---
    painelESP32 = uipanel(tab3, ...
        'Title','Configurações ESP32', ...
        'FontWeight','bold', ...
        'FontSize',12, ...
        'Position',[10 tab3.Position(4)-110 300 80], ...  % posição em normalized units
        'BackgroundColor',[0.95 0.95 0.95], ...
        'BorderType','line');
    
    % Label Porta Serial
    uilabel(painelESP32,'Text','Porta Serial (Ex: COM3):', ...
        'Position',[20 painelESP32.Position(4)-50 150 20]);
    
    % Text field Porta Serial
    portaConfigEdit = uieditfield(painelESP32,'text', ...
        'Position',[180 painelESP32.Position(4)-50 80 20], ...
        'Value','COM5');
    
    % Label Baud Rate
    uilabel(painelESP32,'Text','Baud rate:', ...
        'Position',[20 painelESP32.Position(4)-70 150 20]);
    
    % Combobox Baud Rate
    baudConfigEdit = uidropdown(painelESP32, ...
        'Position',[180 painelESP32.Position(4)-70 100 20], ...
        'Items', {'9600','19200','38400','57600','115200'}, ...
        'Value','115200');


    %% ==============================
    % ABA 1 - COLETA DE DADOS (Layout Ajustado com Painéis)
    % ==============================

    % --- Painel para Parâmetros do Ensaio ---
    painelParametros = uipanel(tab1, ...
        'Title','Parâmetros do Ensaio', ...
        'FontWeight','bold', ...
        'FontSize',12, ...
        'Position',[10 425 450 160], ... % A altura (230) é a nossa referência
        'BackgroundColor',[0.95 0.95 0.95], ...
        'BorderType','line');
    
    % Para tornar o código mais legível, podemos definir os deslocamentos (offsets)
    % a partir do topo do painel para cada linha de controles.
    offsetRow1 = 50;  % Deslocamento para a primeira linha (Tipo de Sinal)
    offsetRow2 = 80;  % Deslocamento para a segunda linha (Duração/Sentido)
    offsetRow3 = 110; % Deslocamento para a terceira linha (Intensidade/Tempo)

    % --- Linha 1 ---
    % Seleção do tipo de sinal
    uilabel(painelParametros,'Text','Tipo de Sinal:','Position',[30 painelParametros.Position(4)-offsetRow1 100 22]);
    tipoSinal = uidropdown(painelParametros,'Items',{'Degrau','Senoide'},...
        'Position',[140 painelParametros.Position(4)-offsetRow1 100 22],...
        'ValueChangedFcn',@atualizarCamposSinal);
    
    % --- Linha 2 (Campos para Degrau) ---
    uilabel(painelParametros,'Text','Duração do Pulso (ms):','Position',[30 painelParametros.Position(4)-offsetRow2 130 22]);
    duracaoEdit = uieditfield(painelParametros,'numeric','Position',[160 painelParametros.Position(4)-offsetRow2 80 22],'Value',500);
    
    sentidoLabel = uilabel(painelParametros,'Text','Sentido:','Position',[250 painelParametros.Position(4)-offsetRow2 80 22]);
    sentidoEdit = uidropdown(painelParametros,...
        'Items',{'Direita','Esquerda'},...
        'ItemsData',{'R','L'},...
        'Value','R',...
        'Position',[330 painelParametros.Position(4)-offsetRow2 100 22]);
    
    % --- Linha 3 ---
    uilabel(painelParametros,'Text','Intensidade (0-255):','Position',[30 painelParametros.Position(4)-offsetRow3 120 22]);
    intensidadeEdit = uieditfield(painelParametros,'numeric','Position',[160 painelParametros.Position(4)-offsetRow3 80 22],'Value',200);
    
    uilabel(painelParametros,'Text','Tempo de coleta (s):','Position',[250 painelParametros.Position(4)-offsetRow3 130 22]);
    tempoEdit = uieditfield(painelParametros,'numeric','Position',[380 painelParametros.Position(4)-offsetRow3 50 22],'Value',10);
    
    % --- Campos exclusivos para senoide (sobrepõem a Linha 2) ---
    % Eles usam o mesmo offset da linha que substituem visualmente.
    freqLabel = uilabel(painelParametros,'Text','Frequência (Hz):','Position',[250 painelParametros.Position(4)-offsetRow2 100 22],'Visible','off');
    freqEdit  = uieditfield(painelParametros,'numeric','Position',[380 painelParametros.Position(4)-offsetRow2 50 22],'Value',1,'Visible','off');

    % --- Painel para Controle da Coleta ---
    painelAcoes = uipanel(tab1, ...
        'Title','Controle da Coleta', ...
        'FontWeight','bold', ...
        'FontSize',12, ...
        'Position',[10 295 450 120], ... % A altura (90) é a nossa referência
        'BackgroundColor',[0.95 0.95 0.95], ...
        'BorderType','line');
    
    % Define a altura dos botões e calcula a posição Y para centralizá-los
    buttonHeight = 40;
    buttonY = (painelAcoes.Position(4) - buttonHeight) / 2; % (90 - 40) / 2 = 25
    
    % Botão 'Iniciar Coleta' posicionado relativamente
    uibutton(painelAcoes,'Text','Iniciar Coleta',...
        'Position',[65 buttonY 150 buttonHeight],... % Usa as variáveis calculadas
        'ButtonPushedFcn',@iniciarColeta,'FontSize',12);
    
    % Botão 'Exportar Dados' posicionado relativamente
    exportarBtn = uibutton(painelAcoes,'Text','Exportar Dados',...
        'Position',[240 buttonY 150 buttonHeight],... % Usa as mesmas variáveis
        'Enable','off','ButtonPushedFcn',@exportarDados,'FontSize',12);

    statusLabel = uilabel(painelAcoes,'Text','Aguardando...','Position',[50 10 350 22],'FontWeight','bold','HorizontalAlignment','center');
    
    % --- Funções de Callback ---
    
    % Função para alternar campos de acordo com o tipo de sinal
    function atualizarCamposSinal(src,~)
        if strcmp(src.Value,'Senoide')
            % Mostra campos da Senoide
            freqLabel.Visible = 'on';
            freqEdit.Visible  = 'on';
            
            % Esconde/Desabilita campos do Degrau
            sentidoEdit.Visible = 'off';
            sentidoLabel.Visible = 'off';
            duracaoEdit.Enable = 'on'; % Duração pode ser útil para ambos
            
        else % Caso seja 'Degrau'
            % Esconde campos da Senoide
            freqLabel.Visible = 'off';
            freqEdit.Visible  = 'off';
            
            % Mostra/Habilita campos do Degrau
            sentidoEdit.Visible = 'on';
            sentidoLabel.Visible = 'on';
            duracaoEdit.Enable = 'on';
        end
    end

    % ==============================
    % FUNÇÕES ABA 1 - COLETA
    % ==============================
    function iniciarColeta(~,~)
   
        porta = portaConfigEdit.Value;
        baud  = str2double(baudConfigEdit.Value);
       
        tempo_total = tempoEdit.Value;
    
        % Parâmetros do sinal
        tipo = tipoSinal.Value;  % 'Degrau' ou 'Senoide'
        duracao = duracaoEdit.Value;
        intensidade = intensidadeEdit.Value;
        sentido = upper(sentidoEdit.Value);
        frequencia = freqEdit.Value;
    
        statusLabel.Text = 'Conectando à serial...';
        drawnow;
    
       
        try
            % Fecha objeto manual se existir
            if ~isempty(sManual) && isvalid(sManual)
                clear sManual;
            end
            s = serialport(porta, baud);
            configureTerminator(s,"LF");
            flush(s);
        catch ME
            uialert(f,sprintf('Falha ao conectar à serial. Porta: %s. Erro: %s', porta, ME.message),'Erro de Conexão');
            statusLabel.Text = 'Erro ao conectar.';
            return;
        end
    
       
        if strcmp(tipo,'Degrau')
            comando = sprintf('D,%d,%d,%s\n', duracao, intensidade, sentido);
        else % Senoide
            comando = sprintf('S,%d,%d,%d\n', intensidade, frequencia, duracao);
        end
        writeline(s, comando);
        pause(0.05);
    
    
        periodo = 10/1000; % 10ms
        t_inicio = tic;
    
        tam_estimado = ceil(tempo_total/periodo) + 100;
        tempo = zeros(1,tam_estimado);
        theta = zeros(1,tam_estimado);
        theta_dot = zeros(1,tam_estimado);
        pos = zeros(1,tam_estimado);
        pos_dot = zeros(1,tam_estimado);
        i = 1;
    
        statusLabel.Text = 'Coletando dados...';
        drawnow;
    
        while toc(t_inicio) < tempo_total
            if s.NumBytesAvailable > 0
                raw_data = readline(s);
                valores = strsplit(strtrim(raw_data),';');
                if length(valores) == 5
                    tempo(i)  = str2double(valores{1});
                    theta(i)  = str2double(valores{2});
                    theta_dot(i)  = str2double(valores{3});
                    pos(i)    = str2double(valores{4});
                    pos_dot(i) = str2double(valores{5});
                    i = i + 1;
                end
            end
            if i > tam_estimado
                warning('Aumentando a pré-alocação dos vetores.');
                tam_estimado = tam_estimado + 5000;
                tempo = [tempo zeros(1,5000)];
                theta = [theta zeros(1,5000)];
                theta_dot = [theta_dot zeros(1,5000)];
                pos = [pos zeros(1,5000)];
                pos_dot = [pos_dot zeros(1,5000)];
            end
        end
    
        clear s; % Fecha serial
    
        % Ajusta vetores ao tamanho real
        tempo = tempo(1:i-1);
        theta = theta(1:i-1);
        theta_dot = theta_dot(1:i-1);
        pos = pos(1:i-1);
        pos_dot = pos_dot(1:i-1);
    
        dados = [tempo' theta' theta_dot' pos' pos_dot'];
    
        f2 = figure('Name','Aquisição de Dados','Position',[600 100 800 500]);
        tPlot = tiledlayout(2,2,'Padding','compact');
        t_plot = tempo;
    
        ax1 = nexttile; plot(ax1,t_plot,pos); title('Posição Carro'); xlabel('t (s)'); grid on; ylabel('Pos (cm)');
        ax2 = nexttile; plot(ax2,t_plot,pos_dot); title('Velocidade Carro'); xlabel('t (s)'); grid on; ylabel('Velocidade (cm/s)');
        ax3 = nexttile; plot(ax3,t_plot,theta); title('Ângulo Pêndulo'); xlabel('t (s)'); grid on; ylabel('Ângulo (°)');
        ax4 = nexttile; plot(ax4,t_plot,theta_dot); title('Velocidade Angular Pêndulo'); xlabel('t (s)'); grid on; ylabel('Velocidade Angular (°/s)');
    
        exportarBtn.Enable = 'on';
        if strcmp(tipo,'Degrau')
            nome_arquivo = sprintf('dados_degrau_I%d_D%d_%s.csv', intensidade, duracao, datestr(now,'HHMMSS'));
        else
            nome_arquivo = sprintf('dados_senoide_I%d_D%d_%s.csv', intensidade, duracao, datestr(now,'HHMMSS'));
        end
        statusLabel.Text = 'Coleta concluída! Pronto para exportar.';
        drawnow;
end


    function exportarDados(~,~)
        if isempty(dados)
            uialert(f,'Nenhum dado disponível para exportar.','Aviso');
            return;
        end
        % Abre a caixa de diálogo para salvar o arquivo
        [file, path] = uiputfile(nome_arquivo, 'Salvar Dados Coletados');
        if isequal(file,0)
           return % Usuário cancelou
        end
        caminho_completo = fullfile(path, file);
        
        writematrix(dados, caminho_completo, 'Delimiter',';');
        uialert(f, sprintf('Dados salvos em:\n%s', caminho_completo), 'Exportação Concluída');
    end

   %% ==============================
    % ABA 2 - CONTROLE MANUAL
    % ==============================
    
    % --- Painel para Conexão Manual ---
    painelConexao = uipanel(tab2, ...
        'Title','Conexão com o Dispositivo', ...
        'FontWeight','bold', ...
        'FontSize',12, ...
        'Position',[10 tab2.Position(4)-155 500 120], ...
        'BackgroundColor',[0.95 0.95 0.95], ...
        'BorderType','line');
    
    % Botão Conectar
    conectarBtn = uibutton(painelConexao,'Text','Conectar',...
        'Position',[100 painelConexao.Position(4)-60 100 30],...
        'ButtonPushedFcn',@conectarManual);
    
    % Botão Desconectar
    desconectarBtn = uibutton(painelConexao,'Text','Desconectar',...
        'Position',[280 painelConexao.Position(4)-60 100 30],...
        'Enable','off',... % Começa desabilitado
        'ButtonPushedFcn',@desconectarManual);
    
    % Label de Status da Conexão
    statusManual = uilabel(painelConexao,'Text','Desconectado',...
        'Position',[150 painelConexao.Position(4)-95 200 20],...
        'FontWeight','bold','HorizontalAlignment','center');
    
    % --- Painel para Controle dos Motores ---
    painelControle = uipanel(tab2, ...
        'Title','Controle Manual do Motor', ...
        'FontWeight','bold', ...
        'FontSize',12, ...
        'Position',[10 painelConexao.Position(4)+220 500 110], ...
        'BackgroundColor',[0.95 0.95 0.95], ...
        'BorderType','line');
    
    % Botão Esquerda
    btnEsq = uibutton(painelControle, 'push', ...
        'Text','⟵ Esquerda', ...
        'Position',[40 painelControle.Position(4)-95 120 60], ... % Posição X ajustada para centralizar o Zerar
        'Enable','off', ...
        'FontSize',14, ...
        'ButtonPushedFcn', @(src,~) acaoImpulso('L'));
        
    % Botão Direita
    btnDir = uibutton(painelControle, 'push', ...
        'Text','Direita ⟶', ...
        'Position',[340 painelControle.Position(4)-95 120 60], ... % Posição X ajustada
        'Enable','off', ...
        'FontSize',14, ...
        'ButtonPushedFcn', @(src,~) acaoImpulso('R'));

    % Botão Zerar (NOVO)
    btnZerar = uibutton(painelControle, 'push', ...
        'Text','Zerar', ...
        'Position',[190 painelControle.Position(4)-95 120 60], ... % Posição central
        'Enable','off', ... % Começa desabilitado
        'FontSize',14, ...
        'ButtonPushedFcn', @(src,~) acaoZerar()); % Chama a nova função

    % ==============================
    % FUNÇÕES ABA 2 - CONTROLE MANUAL
    % ==============================
    function conectarManual(~,~)
        % Usa os valores da aba Configurações
        porta = portaConfigEdit.Value;
        baud  = str2double(baudConfigEdit.Value);
        disp(baud);
        
        % Garante que a porta serial não está sendo usada para coleta
        if ~isempty(sManual) && isvalid(sManual)
            clear sManual; % Apenas para segurança, não deve acontecer
        end
        try
            sManual = serialport(porta, baud);
            configureTerminator(sManual,"LF");
            flush(sManual);
            statusManual.Text = 'Conectado!';
            %start(leituraTimer);
            
            % Habilita e Desabilita botões
            btnEsq.Enable = 'on';
            btnDir.Enable = 'on';
            btnZerar.Enable = 'on'; % NOVO: Habilita o botão Zerar
            conectarBtn.Enable = 'off';
            desconectarBtn.Enable = 'on';
            
            % Desabilita edição na aba Configurações
            portaConfigEdit.Enable = 'off';
            baudConfigEdit.Enable = 'off';
            
        catch ME
            uialert(f,sprintf('Falha ao conectar na serial. Porta: %s. Verifique a porta e se está livre. Erro: %s', porta, ME.message),'Erro de Conexão');
            statusManual.Text = 'Desconectado';
        end
    end
    
    function desconectarManual(~,~)
        if ~isempty(sManual) && isvalid(sManual)
            % Limpa o objeto serialport, liberando a porta COM
            clear sManual; 
            sManual = [];
        end
        
        statusManual.Text = 'Desconectado';
        
        % Habilita e Desabilita botões
        btnEsq.Enable = 'off';
        btnDir.Enable = 'off';
        btnZerar.Enable = 'off'; % NOVO: Desabilita o botão Zerar
        conectarBtn.Enable = 'on';
        desconectarBtn.Enable = 'off';
        
        % Habilita edição na aba Configurações
        portaConfigEdit.Enable = 'on';
        baudConfigEdit.Enable = 'on';
    end
    
    function enviar(cmd)
        if ~isempty(sManual) && isvalid(sManual)
            writeline(sManual, cmd);
        else
            % Se tentar enviar com a porta desconectada
            statusManual.Text = 'Erro: Desconectado!';
            desconectarManual(); % Reinicia o estado dos botões
        end
    end

    function acaoZerar()
        enviar('Z');
    end
    
    function acaoImpulso(sentido)
        % 1. Envia o comando de movimento (L ou R)
        enviar(sentido);
        
        % 2. Adiciona um pequeno atraso para o motor começar a mover
        % O 'P' deve ser enviado rapidamente após o comando de movimento
        pause(0.05); % 50ms (ajuste conforme a necessidade do pulso)
        
        % 3. Envia o comando de parada (P)
        enviar('P');
    end

    function closeFig
    f.CloseRequestFcn = @(src,event)my_closereq(src);
    
        function my_closereq(fig)
            selection = uiconfirm(fig,"Deseja sair da aplicação?",...
                "Confirmação");
            
            switch selection
                case 'OK'
                    delete(fig)
                case 'Cancelar'
                    return
            end
        end
    
    end

    %% -------------------------
    % ABA 04: Controlador lQR + Swing Up energia
    %---------------------------
    
    painelLQR = uipanel(tab4, ...
    'Title','Controlador LQR + Swing Up Baseado em Energia', ...
    'FontWeight','bold', ...
    'FontSize',12, ...
    'Position',[10 tab4.Position(4)-250 300 220], ... 
    'BackgroundColor',[0.95 0.95 0.95], ...
    'BorderType','line');

    uicontrol(painelLQR, 'Style','text', 'String','Tempo de Coleta (s):', ...
        'Position',[10 170 120 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    tempoColetaLQR = uicontrol(painelLQR, 'Style','edit', 'String','30', ...
        'Position',[130 170 50 25], 'BackgroundColor','white');
    
    uicontrol(painelLQR, 'Style','text', 'String','K1 (x)', ...
        'Position',[10 130 50 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    editK1 = uicontrol(painelLQR, 'Style','edit', 'String','-15', ...
        'Position',[60 130 50 25], 'BackgroundColor','white');
    
    uicontrol(painelLQR, 'Style','text', 'String','K4 (θ̇)', ...
        'Position',[130 130 50 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    editK4 = uicontrol(painelLQR, 'Style','edit', 'String','32', ...
        'Position',[180 130 50 25], 'BackgroundColor','white');
    
    uicontrol(painelLQR, 'Style','text', 'String','K2 (θ)', ...
        'Position',[10 100 60 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    editK2 = uicontrol(painelLQR, 'Style','edit', 'String','170', ...
        'Position',[60 100 50 25], 'BackgroundColor','white');
    
    uicontrol(painelLQR, 'Style','text', 'String','K Swing', ...
        'Position',[130 100 50 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    editKswing = uicontrol(painelLQR, 'Style','edit', 'String','45', ...
        'Position',[180 100 50 25], 'BackgroundColor','white');
    
    uicontrol(painelLQR, 'Style','text', 'String','K3 (ẋ)', ...
        'Position',[10 70 50 20], 'HorizontalAlignment','left', 'BackgroundColor',[0.95 0.95 0.95]);
    editK3 = uicontrol(painelLQR, 'Style','edit', 'String','-80', ...
        'Position',[60 70 50 25], 'BackgroundColor','white');
    
    statusLQR = uilabel(painelLQR,'Text','Aguardando comando...','Position',[50 40 200 22],'FontWeight','bold','HorizontalAlignment','center');
    
    btnAtivaLQR = uicontrol(painelLQR, 'Style','pushbutton', 'String','Ativar', ...
        'Position',[95 10 60 25], 'BackgroundColor',[0.7 0.9 0.7], ...
        'Callback',@(src,event) ativarLQR());
    
    btnDesativaLQR = uicontrol(painelLQR, 'Style','pushbutton', 'String','Desativar', ...
        'Position',[165 10 70 25], 'BackgroundColor',[0.9 0.7 0.7], ...
        'Callback',@(src,event) desativarLQR());

    % ==============================
    % FUNÇÕES ABA 4 - CONTROLADOR LQR (COM COLETA)
    % ==============================

    function ativarLQR()
        
        porta = portaConfigEdit.Value;
        baud  = str2double(baudConfigEdit.Value);
       
        % Parâmetros LQR/Swing Up
        K1 = editK1.String;
        K2 = editK2.String;
        K3 = editK3.String;
        K4 = editK4.String;
        Kswing = editKswing.String;
        
        % Definindo o tempo de coleta para a resposta do controlador
        tempo_total = str2double(tempoColetaLQR.String);
        disp(tempo_total);
        statusLQR.Text = 'Conectando à serial...';
        drawnow;
    
        try
            % 1. Abre a porta serial (similar a iniciarColeta)
            sLQR = serialport(porta, baud);
            configureTerminator(sLQR,"LF");
            flush(sLQR);
        catch ME
            uialert(f,sprintf('Falha ao conectar à serial. Porta: %s. Erro: %s', porta, ME.message),'Erro de Conexão');
            statusLQR.Text = 'Erro ao conectar.';
            return;
        end
        
        % 2. Envia o comando LQR/Swing Up
        % Formato: Q,K1,K2,K3,K4,Kswing
        comando = sprintf('Q,%s,%s,%s,%s,%s\n', K1, K2, K3, K4, Kswing);
        writeline(sLQR, comando);
        %pause(0.05);
        
        % 3. Inicia o loop de coleta de dados (similar a iniciarColeta)
        % periodo = 10/1000; % 10ms
        % t_inicio = tic;
        % 
        % tam_estimado = ceil(tempo_total/periodo) + 100;
        % tempo = zeros(1,tam_estimado);
        % theta = zeros(1,tam_estimado);
        % theta_dot = zeros(1,tam_estimado);
        % pos = zeros(1,tam_estimado);
        % pos_dot = zeros(1,tam_estimado);
        % i = 1;
        % 
        % statusLQR.Text = 'Controlador Ativo! Coletando dados ';
        % drawnow;
        % 
        % % Loop de coleta
        % while toc(t_inicio) < tempo_total
        %     if sLQR.NumBytesAvailable > 0
        %         raw_data = readline(sLQR);
        %         valores = strsplit(strtrim(raw_data),';');
        %         if length(valores) == 5
        %             tempo(i)  = str2double(valores{1});
        %             theta(i)  = str2double(valores{2});
        %             theta_dot(i)  = str2double(valores{3});
        %             pos(i)    = str2double(valores{4});
        %             pos_dot(i) = str2double(valores{5});
        %             i = i + 1;
        %         end
        %     end
        %     if i > tam_estimado
        %         % Expande o vetor se necessário
        %         tam_estimado = tam_estimado + 5000;
        %         tempo = [tempo zeros(1,5000)];
        %         theta = [theta zeros(1,5000)];
        %         theta_dot = [theta_dot zeros(1,5000)];
        %         pos = [pos zeros(1,5000)];
        %         pos_dot = [pos_dot zeros(1,5000)];
        %     end
        % end
        % 
        % % 4. Encerra a coleta e plota os resultados
        % 
        % % Envia o comando 'X' para desativar o controlador no ESP32 antes de fechar
        % writeline(sLQR, 'X\n'); 
        % pause(0.1); 
        
        %clear sLQR; % Fecha serial
    
        % Ajusta vetores ao tamanho real
        % tempo = tempo(1:i-1);
        % theta = theta(1:i-1);
        % theta_dot = theta_dot(1:i-1);
        % pos = pos(1:i-1);
        % pos_dot = pos_dot(1:i-1);
        % 
        % % Salva globalmente (opcional, mas bom para exportar)
        % dados = [tempo' theta' theta_dot' pos' pos_dot'];
        % 
        % % Plota os dados (similar a iniciarColeta)
        % f2 = figure('Name','Resposta do Controlador LQR','Position',[600 100 800 500]);
        % tPlot = tiledlayout(2,2,'Padding','compact');
        % t_plot = tempo;
        % 
        % ax1 = nexttile; plot(ax1,t_plot,pos); title('Posição Carro'); xlabel('t (s)'); grid on; ylabel('Pos (cm)');
        % ax2 = nexttile; plot(ax2,t_plot,pos_dot); title('Velocidade Carro'); xlabel('t (s)'); grid on; ylabel('Velocidade (cm/s)');
        % ax3 = nexttile; plot(ax3,t_plot,theta); title('Ângulo Pêndulo'); xlabel('t (s)'); grid on; ylabel('Ângulo (°)');
        % ax4 = nexttile; plot(ax4,t_plot,theta_dot); title('Velocidade Angular Pêndulo'); xlabel('t (s)'); grid on; ylabel('Velocidade Angular (°/s)');
        % 
        % statusLQR.Text = 'Coleta e Controle Concluídos! Dados Plotados.';
        % 
        % % Define o nome do arquivo para exportação
        % nome_arquivo = sprintf('dados_LQR_K1_%s_K2_%s_%s.csv', K1, K2, datestr(now,'HHMMSS'));
        % % Você precisará de um botão 'Exportar' na Aba 4 que chame a função exportarDados
    end
    
    function desativarLQR()
        writeline(sLQR, 'X\n');        
        a% Esta função só é útil se a coleta for feita em background (com um timer), 
        % mas no seu modelo atual, a coleta roda no thread principal até o fim.
    end

    
    %closeFig;
end