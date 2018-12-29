%% Analysis of PNC Data
inch_to_mm = 25.4;
export_figures = 1;
axis_labels = {'X', 'Y', 'Z', 'A', 'B'};
axis_scale = [25.4, 25.4, 25.4, 1, 1];
axis_units = {'mm', 'mm', 'mm', 'deg', 'deg'};
figure_names = {'HeadTopXAxis', 'HeadTopYAxis', 'HeadTopZAxis', 'HeadTopAAxis', 'HeadTopBAxis'};
figure_subnames = {'', '_Detail1', '_Detail2'};
titles = {'%s', '20 Second Detail of %s', '3 Second Detail of %s'};
detail_ranges = {[15 35; 19 22], [405 435; 422 425], [235 255; 167 170], [285 305; 198 201], [105 135; 113 116]};

%%
% Head Top Pass
commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_COMMANDED_SERVO_POSITIONS.csv',1);
commanded_positions = commanded_positions(:,[1 2 5 3 6 7]);
stepgen_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_STEPGEN_FEEDBACK_POSITIONS.csv',1);
stepgen_positions = sortrows(stepgen_positions(:,[1 2 5 3 6 7]),1);

encoder_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_ENCODER_FEEDBACK_POSITIONS.csv',1);
encoder_positions = encoder_positions(:,[1 2 5 3 6 7]);

tcq_length = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_HIGHRES_TC_QUEUE_LENGTH.csv',1);
pid_delays = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_NETWORK_PID_DELAYS.csv',1);

%Data arrays
toolpath_commands = {commanded_positions};
toolpath_estimates = {stepgen_positions};
toolpath_actuals = {encoder_positions};

%% Position plots
close all;
t_min = 28; t_max = 530; encoder_offset = (.895-.8775);
for ax_num = 1:5
    break
    figure(ax_num); clf;
    hold on;
    plot(commanded_positions(:,1)-t_min, commanded_positions(:,1+ax_num)*axis_scale(ax_num),'b-.')
    plot(stepgen_positions(:,1)-t_min, stepgen_positions(:,1+ax_num)*axis_scale(ax_num),'r--')
    plot(encoder_positions(:,1)-t_min+encoder_offset, encoder_positions(:,1+ax_num)*axis_scale(ax_num),'k')
    hold off;

    title({[axis_labels{ax_num} '-Axis Position Progression'], 'for Head Top Pass'}, 'FontName', 'Times', 'FontSize', 16), 
    ax = gca;
    ax.FontName = 'Times';
    ax.FontSize = 12;
    xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 16); ylabel(sprintf('%s-Axis Position (%s)', axis_labels{ax_num}, axis_units{ax_num}), 'FontName', 'Times', 'FontSize', 16);
    xlim([0 t_max])

    legend('Commanded Position', 'Estimated Position', 'Actual Position', 'Location', 'SouthEast');
end

%% Derivative plots
close all;
t_min = 28; t_max = 500; encoder_offset = (.895-.8775); %command_offset = 3.7-145.175+144.8+.32;
command_offset = 0;
for ax_num = 1:5
    command_time = commanded_positions(:,1)-t_min+command_offset;
    dctdt = diff(command_time);
    d2cdt2 = diff(dctdt);

    stepgen_time = stepgen_positions(:,1)-t_min;
    dstdt = diff(stepgen_time);
    d2stdt2 = diff(dstdt);

    encoder_time = encoder_positions(:,1)-t_min+encoder_offset;
    detdt = diff(encoder_time);
    d2etdt2 = diff(detdt);

    commanded_position = commanded_positions(:,ax_num+1)*axis_scale(ax_num);
    windowsize = 20;
    dcdt = filter(1/windowsize*ones(1,windowsize), 1, diff(commanded_position));
    %dcdt = diff(commanded_position);
    d2cdt2 = diff(dcdt);

    stepgen_axis_position = stepgen_positions(:,ax_num+1)*axis_scale(ax_num);
    dspdt = diff(stepgen_axis_position);
    d2spdt2 = diff(dspdt);

    encoder_axis_position = encoder_positions(:,ax_num+1)*axis_scale(ax_num);
    dedt = diff(encoder_axis_position);
    d2edt2 = diff(dedt);
    
    limits = {[0 t_max], detail_ranges{ax_num}(1,:), detail_ranges{ax_num}(2,:)};
    for fig_num = 1:3
        figure((ax_num-1)*length(limits)+fig_num); clf;
        
        subplot(2,1,1);
        hold on;
        plot(command_time, commanded_position,'b-.')
        plot(stepgen_time, stepgen_axis_position,'r--')
        plot(encoder_time, encoder_axis_position,'k')
        hold off;

        %title({[axis_labels{ax_num} '-Axis Position Progression'], 'for Head Top Pass'}, 'FontName', 'Times', 'FontSize', 16), 
        title(sprintf(titles{fig_num}, 'Position Progression'), 'FontName', 'Times', 'FontSize', 14), 
        ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
        xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 12); ylabel(sprintf('%s-Axis Position (%s)', axis_labels{ax_num}, axis_units{ax_num}), 'FontName', 'Times', 'FontSize', 12);
        xlim(limits{fig_num})

        legend('Commanded Position', 'Estimated Position', 'Actual Position', 'Location', 'Best');

        %Velocity
        subplot(2,1,2);
        hold on;
        plot(command_time(1:end-1), sgolayfilt(dcdt*1000,3,7), 'b');
        %plot(command_time(1:end-1), sgolayfilt(dcdt./dctdt,5,19), 'b');
        plot(stepgen_time(1:end-1), sgolayfilt(dspdt./dstdt,3,7), 'r--')
        plot(encoder_time(1:end-1), dedt./detdt, 'k');
        hold off;

        %title({[axis_labels{ax_num} '-Axis Velocity Progression'], 'for Head Top Pass'}, 'FontName', 'Times', 'FontSize', 14)
        title(sprintf(titles{fig_num}, 'Velocity Progression'),'FontName', 'Times', 'FontSize', 14);
        ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
        xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 12); ylabel(sprintf('%s-Axis Velocity (%s/s)', axis_labels{ax_num}, axis_units{ax_num}), 'FontName', 'Times', 'FontSize', 12);
        xlim(limits{fig_num})
        legend('Commanded Velocity', 'Estimated Velocity', 'Actual Velocity', 'Location', 'Best');

    %     %Acceleration
    %     subplot(3,1,3);
    %     hold on;
    %     plot(command_time(1:end-2), sgolayfilt(d2cdt2*1000,3,7), 'b');
    %     plot(stepgen_time(1:end-2), sgolayfilt(d2spdt2./dstdt(1:end-1),3,19), 'r--')
    %     %plot(encoder_time(1:end-2), sgolayfilt(d2edt2./d2etdt2,3,7), 'k');
    %     hold off;
        set(gcf,'Position', [100 100 800 800]);
    end
end


%%
if export_figures
    for k = 1:length(figure_names)
        for ks = 1:length(figure_subnames)
            figure((k-1)*length(figure_subnames)+ks);
            (k-1)*length(figure_subnames)+ks
            set(gcf, 'PaperUnits', 'inches');
            set(gcf, 'PaperSize', [6 8]);
            set(gcf, 'PaperPositionMode', 'manual');
            set(gcf, 'PaperPosition', [0 0 6 8]);
            print([figure_names{k} figure_subnames{ks}],'-dpdf');
        end
    end
end

%%
%3D Plots
% close all;
% 
% for k = 1:1
%     plot3(toolpath_commands{k}(:,2), toolpath_commands{k}(:,3), toolpath_commands{k}(:,3))
% end
