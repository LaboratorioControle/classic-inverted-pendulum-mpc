function realTimePlot(s)

    Fs = 100;        % 100 Hz (10 ms)
    Tjanela = 5;     % segundos
    N = Fs*Tjanela;

    tbuf = nan(N,1);
    theta_buf = nan(N,1);
    x_buf = nan(N,1);

    % === FIGURA ===
    fig = figure('Name','Pêndulo em Tempo Real','NumberTitle','off');

    ax1 = subplot(2,1,1);
    p1 = plot(ax1,nan,nan,'b');
    ylabel('\theta (deg)')
    ylim([0 360]);
    grid on

    ax2 = subplot(2,1,2);
    p2 = plot(ax2,nan,nan,'r');
    ylabel('x (cm)')
    ylim([-30 30]);
    xlabel('Tempo (s)')
    grid on

    t0 = [];

    while ishandle(fig)

        waitHeader(s);

        t_ms      = read(s,1,"uint32");
        theta_deg = read(s,1,"single");
        theta_dot = read(s,1,"single");
        x_cm      = read(s,1,"single");
        x_dot_cm  = read(s,1,"single");

        if isempty(t0)
            t0 = t_ms;
        end

        t = (t_ms - t0)/1000;

        % buffer circular
        tbuf = [tbuf(2:end); t];
        theta_buf = [theta_buf(2:end); theta_deg];
        x_buf = [x_buf(2:end); x_cm];

        set(p1,'XData',tbuf,'YData',theta_buf);
        set(p2,'XData',tbuf,'YData',x_buf);

        drawnow limitrate
    end
end
