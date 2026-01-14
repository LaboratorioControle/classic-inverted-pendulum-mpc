pasta = "ESP32\MPC_Inverted_Pendulum\lib\MPC\";

arquivos = dir(fullfile(pasta, "*.csv"));

for i = 1:length(arquivos)

    nome_arquivo = arquivos(i).name;          % Ex: 'H.csv'
    cam_completo = fullfile(pasta, nome_arquivo);

    nome_matriz = erase(nome_arquivo, ".csv"); % Ex: 'H'

    % Atribuição dinâmica em MPC
    MPC.(nome_matriz) = readmatrix(cam_completo);

    fprintf('✅ MPC.%s carregada com sucesso\n', nome_matriz);

end
