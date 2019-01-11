%% Analysis of PNC Data
inch_to_mm = 25.4;
export_figures = 1;
axis_labels = {'X', 'Y', 'Z', 'A', 'B'};
axis_scale = [25.4, 25.4, 25.4, 1, 1];
axis_units = {'mm', 'mm', 'mm', 'deg', 'deg'};

T_min = [28, 28, 33, 22];
T_max = [500, 495, 1053, 156];
E_offset = [(.895-.8775)+.006, (113.99-113.976)+.0072, (369.572-369.55)+(946.3789-946.3665), (27.585-27.571)];
C_offset = [0, 0, 0, 0];
pass_names = {'HeadTop', 'HeadBottom', 'CHolderTop', 'CHolderBottom'};
figure_names = {'XAxis', 'YAxis', 'ZAxis', 'AAxis', 'BAxis'};
figure_subnames = {'', '_Detail1', '_Detail2'};
titles = {'%s', '20 Second Detail of %s', '3 Second Detail of %s'};
%detail_ranges = {[15 35; 19 22], [405 435; 422 425], [235 255; 167 170], [285 305; 198 201], [105 135; 113 116]};
detail_ranges = {{[15 35; 19 22], [405 435; 422 425], [235 255; 167 170], [285 305; 198 201], [105 135; 113 116]}; 
    {[290 310; 305 308], [330 350; 334 337], [95 115; 108 111], [230 250; 244 247], [290 310; 306 309]};
    {[80 100; 86 89], [165 185; 178 181], [355 375; 369 372], [955 975; 970 973], [920 950; 944 947]};
    {[140 160; 152 155], [120 140; 128 131], [26 46; 27 30], [65 85; 82 85], [55 75; 61 64]}};

%%
% Head Top Pass
commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_COMMANDED_SERVO_POSITIONS.csv',1);
headtop_commanded_positions = commanded_positions(:,[1 2 5 3 6 7]);
stepgen_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_STEPGEN_FEEDBACK_POSITIONS.csv',1);
headtop_stepgen_positions = sortrows(stepgen_positions(:,[1 2 5 3 6 7]),1);
encoder_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_ENCODER_FEEDBACK_POSITIONS.csv',1);
headtop_encoder_positions = encoder_positions(:,[1 2 5 3 6 7]);
headtop_tcq_length = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_HIGHRES_TC_QUEUE_LENGTH.csv',1);
headtop_pid_delays = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_NETWORK_PID_DELAYS.csv',1);
headtop_pull_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_raw_pull_times.csv');
headtop_push_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_top_try2_raw_push_times.csv');

% Head Top SP Data
sp_commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\sp_headtop_tp_commands.csv');
sp_commanded_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\sp_headtop_tp_command_times.csv');
headtop_sp_commanded_positions = [sp_commanded_times(1:length(sp_commanded_positions)) sp_commanded_positions];

% Head Bottom Pass
commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_COMMANDED_SERVO_POSITIONS.csv',1);
headbottom_commanded_positions = commanded_positions(:,[1 2 5 3 6 7]);
stepgen_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_STEPGEN_FEEDBACK_POSITIONS.csv',1);
headbottom_stepgen_positions = sortrows(stepgen_positions(:,[1 2 5 3 6 7]),1);
encoder_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_ENCODER_FEEDBACK_POSITIONS.csv',1);
headbottom_encoder_positions = encoder_positions(:,[1 2 5 3 6 7]);
headbottom_tcq_length = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_HIGHRES_TC_QUEUE_LENGTH.csv',1);
headbottom_pid_delays = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_NETWORK_PID_DELAYS.csv',1);
headbottom_pull_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_raw_pull_times.csv');
headbottom_push_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Head Data\head_bottom_raw_push_times.csv');

% CHolder Top Pass
commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_COMMANDED_SERVO_POSITIONS.csv',1);
choldertop_commanded_positions = commanded_positions(:,[1 2 5 3 6 7]);
stepgen_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_STEPGEN_FEEDBACK_POSITIONS.csv',1);
choldertop_stepgen_positions = sortrows(stepgen_positions(:,[1 2 5 3 6 7]),1);
encoder_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_ENCODER_FEEDBACK_POSITIONS.csv',1);
choldertop_encoder_positions = encoder_positions(:,[1 2 5 3 6 7]);
choldertop_tcq_length = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_HIGHRES_TC_QUEUE_LENGTH.csv',1);
choldertop_pid_delays = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_NETWORK_PID_DELAYS.csv',1);
choldertop_pull_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_raw_pull_times.csv');
choldertop_push_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_top_raw_push_times.csv');

% CHolder Bottom Pass
commanded_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_COMMANDED_SERVO_POSITIONS.csv',1);
cholderbottom_commanded_positions = commanded_positions(:,[1 2 5 3 6 7]);
stepgen_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_STEPGEN_FEEDBACK_POSITIONS.csv',1);
cholderbottom_stepgen_positions = sortrows(stepgen_positions(:,[1 2 5 3 6 7]),1);
encoder_positions = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_ENCODER_FEEDBACK_POSITIONS.csv',1);
cholderbottom_encoder_positions = encoder_positions(:,[1 2 5 3 6 7]);
cholderbottom_tcq_length = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_HIGHRES_TC_QUEUE_LENGTH.csv',1);
cholderbottom_pid_delays = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_NETWORK_PID_DELAYS.csv',1);
cholderbottom_pull_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_raw_pull_times.csv');
cholderbottom_push_times = csvread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Candleholder Data\candleholder_bottom_good_raw_push_times.csv');


%Data arrays
toolpath_commands = {headtop_commanded_positions, headbottom_commanded_positions, choldertop_commanded_positions, cholderbottom_commanded_positions};
toolpath_estimates = {headtop_stepgen_positions, headbottom_stepgen_positions, choldertop_stepgen_positions, cholderbottom_stepgen_positions};
toolpath_actuals = {headtop_encoder_positions, headbottom_encoder_positions, choldertop_encoder_positions, cholderbottom_encoder_positions};
tcq_lengths = {headtop_tcq_length, headbottom_tcq_length, choldertop_tcq_length, cholderbottom_tcq_length};
pid_delays = {headtop_pid_delays, headbottom_pid_delays, choldertop_pid_delays, cholderbottom_pid_delays};
pull_times = {headtop_pull_times, headbottom_pull_times, choldertop_pull_times, cholderbottom_pull_times};
push_times = {headtop_push_times, headbottom_push_times, choldertop_push_times, cholderbottom_push_times};

% %% Position plots
% close all;
% t_min = 28; t_max = 530; encoder_offset = (.895-.8775);
% for ax_num = 1:5
%     break
%     figure(ax_num); clf;
%     hold on;
%     plot(commanded_positions(:,1)-t_min, commanded_positions(:,1+ax_num)*axis_scale(ax_num),'b-.')
%     plot(stepgen_positions(:,1)-t_min, stepgen_positions(:,1+ax_num)*axis_scale(ax_num),'r--')
%     plot(encoder_positions(:,1)-t_min+encoder_offset, encoder_positions(:,1+ax_num)*axis_scale(ax_num),'k')
%     hold off;
% 
%     title({[axis_labels{ax_num} '-Axis Position Progression'], 'for Head Top Pass'}, 'FontName', 'Times', 'FontSize', 16), 
%     ax = gca;
%     ax.FontName = 'Times';
%     ax.FontSize = 12;
%     xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 16); ylabel(sprintf('%s-Axis Position (%s)', axis_labels{ax_num}, axis_units{ax_num}), 'FontName', 'Times', 'FontSize', 16);
%     xlim([0 t_max])
% 
%     legend('Commanded Position', 'Estimated Position', 'Actual Position', 'Location', 'SouthEast');
% end

%% Derivative plots
for pass = 1:4
    close all;
    t_min = T_min(pass); t_max = T_max(pass); encoder_offset = E_offset(pass); command_offset = C_offset(pass);
    commanded_positions = toolpath_commands{pass};
    stepgen_positions = toolpath_estimates{pass};
    encoder_positions = toolpath_actuals{pass};
    detail_range = detail_ranges{pass};
    %t_min = 28; t_max = 500; encoder_offset = (.895-.8775); %command_offset = 3.7-145.175+144.8+.32;
    %command_offset = 0;
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

        limits = {[0 t_max], detail_range{ax_num}(1,:), detail_range{ax_num}(2,:)};
        for fig_num = 1:3
            figure((ax_num-1)*length(limits)+fig_num); clf;

            subplot(2,1,1);
            hold on;
            plot(command_time, commanded_position,'b-.')
            plot(stepgen_time, stepgen_axis_position,'r--')
            plot(encoder_time, encoder_axis_position,'k')
            hold off;

            %title({[axis_labels{ax_num} '-Axis Position Progression'], 'for Head Top Pass'}, 'FontName', 'Times', 'FontSize', 16), 
            title(sprintf(titles{fig_num}, 'Motion Profile'), 'FontName', 'Times', 'FontSize', 12), 
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
            %title(sprintf(titles{fig_num}, 'Velocity Progression'),'FontName', 'Times', 'FontSize', 12);
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

    if export_figures
        for k = 1:length(figure_names)
            for ks = 1:length(figure_subnames)
                figure((k-1)*length(figure_subnames)+ks);
                (k-1)*length(figure_subnames)+ks;
                set(gcf, 'PaperUnits', 'inches');
                set(gcf, 'PaperSize', [6 8]);
                set(gcf, 'PaperPositionMode', 'manual');
                set(gcf, 'PaperPosition', [0 0 6 8]);
                print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{pass} figure_names{k} figure_subnames{ks}],'-dpdf');
            end
        end
    end
end

%% Buffer Level Plots
close all;
export_figures = 1;
data_start_indices = [102 111 111 109];
data_end_indices = [8507 7997 17850 2802];
for pass = 1:4
    tcq_length = tcq_lengths{pass};
    [~, start_time_index] = min(abs(tcq_length(:,1)-(tcq_length(find(tcq_length(:,2),1),1)-5)));
    start_time = tcq_length(start_time_index,1);
    tcq_time = tcq_length(start_time_index:end,1)-start_time;
    %tcq_data = sgolayfilt(tcq_length(start_time_index:end,2),3,27);
    tcq_data = tcq_length(start_time_index:end,2);
    end_time = tcq_time(end,1);
    %start_time = tcq_length(find(tcq_length(:,2),1),1)
    
    figure(pass);
    plot(tcq_time, tcq_data,'b', [0 end_time], [500 500],'r--', [0 end_time], [2000 2000], 'k--')
    xlim([0 tcq_time(end,1)]);
    if max(tcq_length(:,2)) < 2000
        ylim([0 2100])
    end
    
    %title(sprintf(titles{fig_num}, 'Velocity Progression'),'FontName', 'Times', 'FontSize', 14);
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 12); ylabel(sprintf('Trajectory Buffer Fill Level (ms)', axis_labels{ax_num}, axis_units{ax_num}), 'FontName', 'Times', 'FontSize', 12);
    %xlim(limits{fig_num})
    legend('Buffer Fill Level', 'Setpoint', 'Buffer Limit', 'Location', 'Best');
    
    fprintf('For pass %i, the MIN buffer level was %i\n', pass, min(tcq_data(data_start_indices(pass):data_end_indices(pass))))
    fprintf('For pass %i, the MAX buffer level was %i\n', pass, max(tcq_data(1:end)))
    fprintf('For pass %i, the MEAN buffer level was %i\n', pass, mean(tcq_data(data_start_indices(pass):data_end_indices(pass))))
 
end

if export_figures
    for k = 1:length(pass_names)
        figure(k)
        set(gcf, 'PaperUnits', 'inches');
        set(gcf, 'PaperSize', [6 2.5]);
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperPosition', [0 0 6 2.5]);
        print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{k} '_BufferLevel'],'-dpdf');
    end
end

%% Buffer Delay Plots
close all;
export_figures = 1;
for pass = 1:4
    buffer_data = pid_delays{pass};
    buffer_tx_time = buffer_data(:,1)-buffer_data(1,1);
    buffer_delays = buffer_data(:,2)*1000;
    %[~, start_time_index] = min(abs(tcq_length(:,1)-(tcq_length(find(tcq_length(:,2),1),1)-5)));
    %start_time = tcq_length(start_time_index,1);
    %tcq_time = tcq_length(start_time_index:end,1)-start_time;
    %tcq_data = sgolayfilt(tcq_length(start_time_index:end,2),3,27);
    %tcq_data = tcq_length(start_time_index:end,2);
    %end_time = tcq_time(end,1);
    %start_time = tcq_length(find(tcq_length(:,2),1),1)
    
    figure(pass);
    hold on;
    plot(buffer_tx_time, buffer_delays,'b')
    windowsize = 31;
    extended_bd_data = [buffer_delays(1:windowsize); buffer_delays];
    moving_avg_data = filtfilt(1/windowsize*ones(windowsize,1),1,extended_bd_data);
    plot(buffer_tx_time, moving_avg_data(windowsize+1:end),'r','LineWidth',1)
    xlim([0 buffer_tx_time(end)]);
    ylim([0 65]);
%     xlim([0 tcq_time(end,1)]);
%     if max(tcq_length(:,2)) < 2000
%         ylim([0 2100])
%     end
    
    %title(sprintf(titles{fig_num}, 'Velocity Progression'),'FontName', 'Times', 'FontSize', 14);
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 12); ylabel('Transmission Delay (ms)', 'FontName', 'Times', 'FontSize', 12);
    %xlim(limits{fig_num})
    legend('Transmission Delay', 'Filtered Transmission Delay', 'Location', 'Best');
 
end

if export_figures
    for k = 1:length(pass_names)
        figure(k)
        set(gcf, 'PaperUnits', 'inches');
        set(gcf, 'PaperSize', [6 2]);
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperPosition', [0 0 6 2]);
        print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{k} '_BufferDelay'],'-dpdf');
    end
end

%% Encoder Frequency Plots
close all;
export_figures = 1;
toolpath_start_indices = {[2041 17042 2780], [2041 15544 2506], [2041 18552 2943], [2041 11959 1842]};
toolpath_end_indices = {[493472 259722 38384], [466632 243750 37047], [1043701 531758 77363], [159788 89900 13383]};
for pass = 1:4
    encoder_times = toolpath_actuals{pass}(toolpath_start_indices{pass}(3):toolpath_end_indices{pass}(3),1);
    encoder_times(:,1) = encoder_times(:,1)-encoder_times(1,1);
    
    figure(pass);
    hold on;
    plot(encoder_times(1:end-1), sgolayfilt(1./diff(encoder_times),3,5),'b')
    windowsize = 200;
    extended_freq_data = [encoder_times(1:windowsize); encoder_times];
    moving_avg_data = filter(1/windowsize*ones(windowsize,1),1,1./diff(extended_freq_data));
    %moving_avg_data = filtfilt(1/windowsize*ones(windowsize,1),1,1./diff(extended_freq_data));
    %moving_avg_data = filtfilt(1/windowsize*ones(windowsize,1),1,1./diff(encoder_times));
    plot(encoder_times(1:end), moving_avg_data(windowsize:end),'r','LineWidth',1)
    hold off;
    xlim([0 encoder_times(end)]);
    legend('Instantaneous Encoder Read Frequency', sprintf('%i Sample Moving Average',windowsize), 'Location','Best');
    
%     xlim([0 tcq_time(end,1)]);
%     if max(tcq_length(:,2)) < 2000
%         ylim([0 2100])
%     end
    
    %title(sprintf(titles{fig_num}, 'Velocity Progression'),'FontName', 'Times', 'FontSize', 14);
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    xlabel('Time (s)', 'FontName', 'Times', 'FontSize', 12); ylabel('Encoder Read Frequency (Hz)', 'FontName', 'Times', 'FontSize', 12);
    %xlim(limits{fig_num})
    %legend('Buffer Fill Level', 'Setpoint', 'Buffer Limit', 'Location', 'Best');
 
end

if export_figures
    for k = 1:length(pass_names)
        figure(k)
        set(gcf, 'PaperUnits', 'inches');
        set(gcf, 'PaperSize', [6 2.5]);
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperPosition', [0 0 6 2.5]);
        print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{k} '_EncoderFreq'],'-dpdf');
    end
end

%% Database Performance
close all;
export_figures = 1;
toolpath_start_indices = {[2041 17042 2780], [2041 15544 2506], [2041 18552 2943], [2041 11959 1842]};
toolpath_end_indices = {[493472 259722 38384], [466632 243750 37047], [1043701 531758 77363], [159788 89900 13383]};
for pass = 1:4
    end_time = toolpath_estimates{pass}(end,1);
    pull_time = pull_times{pass}*1000;
    push_time = push_times{pass}*1000;
    
    avg_pulls_per_sec = length(pull_time)/end_time
    avg_pushes_per_sec = length(push_time)/end_time
    
%     encoder_times = toolpath_actuals{pass}(toolpath_start_indices{pass}(3):toolpath_end_indices{pass}(3),1);
%     encoder_times(:,1) = encoder_times(:,1)-encoder_times(1,1);
    
    figure(pass);
    subplot(2,1,1);
    hold on;
    windowsize = 20;
    plot(pull_time,'b');
    %plot(filtfilt(1/windowsize*ones(windowsize,1),1,pull_time),'r','LineWidth',1);
    plot(filter(1/windowsize*ones(windowsize,1),1,pull_time),'r','LineWidth',1);
    hold off;
    xlim([0,length(pull_time)]); ylim([0 20]);
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    %xlabel('Request Count', 'FontName', 'Times', 'FontSize', 12); 
    ylabel('Execution Time (ms)', 'FontName', 'Times', 'FontSize', 12);
    legend('Instantaneous Pull Duration', sprintf('%i Sample Moving Average',windowsize), 'Location','Best');
    
    subplot(2,1,2);
    hold on;
    plot(push_time,'b');
    windowsize = 20;
    plot(filtfilt(1/windowsize*ones(windowsize,1),1,push_time),'r','LineWidth',1);
    hold off;
    xlim([0,length(push_time)]); ylim([0 20]);
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    xlabel('Request Count', 'FontName', 'Times', 'FontSize', 12); ylabel('Execution Time (ms)', 'FontName', 'Times', 'FontSize', 12);
    legend('Instantaneous Push Duration', sprintf('%i Sample Moving Average',windowsize), 'Location','Best');
    %plot(encoder_times(1:end-1), sgolayfilt(1./diff(encoder_times),3,5),'b')
    
    %extended_freq_data = [encoder_times(1:windowsize); encoder_times];
    %moving_avg_data = filter(1/windowsize*ones(windowsize,1),1,1./diff(extended_freq_data));
    %moving_avg_data = filtfilt(1/windowsize*ones(windowsize,1),1,1./diff(extended_freq_data));
    %moving_avg_data = filtfilt(1/windowsize*ones(windowsize,1),1,1./diff(encoder_times));
    %plot(encoder_times(1:end), moving_avg_data(windowsize:end),'r','LineWidth',1)
    
    %xlim([0 encoder_times(end)]);
%     legend('Insntaneous Encoder Read Frequency', sprintf('%i Sample Moving Average',windowsize), 'Location','Best');

end

if export_figures
    for k = 1:length(pass_names)
        figure(k)
        set(gcf, 'PaperUnits', 'inches');
        set(gcf, 'PaperSize', [6 3.5]);
        set(gcf, 'PaperPositionMode', 'manual');
        set(gcf, 'PaperPosition', [0 0 6 3.5]);
        print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{k} '_DB'],'-dpdf');
    end
end

%%
%3D Plots
close all;
export_figures = 0;
wpc_subnames = {'' '_Detail'};
work_translation = [0 0 0.3];
tool_translation = [0 0 2];
toolpath_start_indices = {[2041 17042 2780], [2041 15544 2506], [2041 18552 2943], [2041 11959 1842]};
toolpath_end_indices = {[493472 259722 38384], [466632 243750 37047], [1043701 531758 77363], [159788 89900 13383]};
reorient_end_indices = [1, 1, 1, 1];
deviation_start_points = [1 1 1 1];
deviation_points = [5 5 5 5];
view_configs = {{[46 22], [46, 22]}, {[45 74], [55 36]}, {[113 82], [46 22]}, {[-159 24], [46 22]}};
zooms = [42 58 100 40];

for pass = 1:4
    commanded_positions = toolpath_commands{pass};
    commanded_positions = commanded_positions(toolpath_start_indices{pass}(1):toolpath_end_indices{pass}(1),:);
    commanded_positions(:,1) = commanded_positions(:,1)-commanded_positions(1,1);
    %commanded_positions(:,1) = [linspace(stepgen_positions(1,1),stepgen_positions(end,1),length(commanded_positions(:,1)))]';
    %commanded_positions(:,2:4) = commanded_positions(:,2:4)*25.4;
    [~, ia, ~] = unique(commanded_positions(:,1), 'rows');
    commanded_positions = commanded_positions(ia,:);
    %commanded_positions(1,:) = 
    
    stepgen_positions = toolpath_estimates{pass};
    stepgen_positions = stepgen_positions(toolpath_start_indices{pass}(2):toolpath_end_indices{pass}(2),:);
    stepgen_positions(:,1) = stepgen_positions(:,1)-stepgen_positions(1,1);
    %stepgen_positions(:,2:4) = stepgen_positions(:,2:4)*25.4;
    [~, ia, ~] = unique(stepgen_positions(:,1), 'rows');
    stepgen_positions = stepgen_positions(ia,:);
    %commanded_positions(:,1) = [linspace(stepgen_positions(1,1),stepgen_positions(end,1),length(commanded_positions(:,1)))]';
    
    encoder_positions = toolpath_actuals{pass};
    encoder_positions = encoder_positions(toolpath_start_indices{pass}(3):toolpath_end_indices{pass}(3),:);
    encoder_positions(:,1) = encoder_positions(:,1)-encoder_positions(1,1);
    %encoder_positions(:,2:4) = encoder_positions(:,2:4)*25.4;
	[~, ia, ~] = unique(encoder_positions(:,1), 'rows');
    encoder_positions = encoder_positions(ia,:);
    
    FK_commands = FK(commanded_positions(:,2:end), tool_translation, work_translation);
    FK_stepgen = FK(stepgen_positions(:,2:end), tool_translation, work_translation);
    FK_encoder = FK(encoder_positions(:,2:end), tool_translation, work_translation);
    
    FK_commands(:, 1:3) = FK_commands(:,1:3)*25.4;
    FK_stepgen(:, 1:3) = FK_stepgen(:,1:3)*25.4;
    FK_encoder(:, 1:3) = FK_encoder(:,1:3)*25.4;
    
%     FK_interp_stepgen = [interp1(stepgen_positions(:,1), FK_stepgen(:,1), commanded_positions(:,1),'spline'),...
%         interp1(stepgen_positions(:,1), FK_stepgen(:,2), commanded_positions(:,1),'spline'),...
%         interp1(stepgen_positions(:,1), FK_stepgen(:,3), commanded_positions(:,1),'spline'),...
%         interp1(stepgen_positions(:,1), FK_stepgen(:,4), commanded_positions(:,1),'spline'),...
%         interp1(stepgen_positions(:,1), FK_stepgen(:,5), commanded_positions(:,1),'spline')];
        
    FK_interp_encoder = [interp1(encoder_positions(:,1), FK_encoder(:,1), stepgen_positions(:,1),'spline'),...
        interp1(encoder_positions(:,1), FK_encoder(:,2), stepgen_positions(:,1),'spline'),...
        interp1(encoder_positions(:,1), FK_encoder(:,3), stepgen_positions(:,1),'spline'),...
        interp1(encoder_positions(:,1), FK_encoder(:,4), stepgen_positions(:,1),'spline'),...
        interp1(encoder_positions(:,1), FK_encoder(:,5), stepgen_positions(:,1),'spline')];
    
    cmd_est_nn_ndx = knnsearch(FK_commands, FK_stepgen);
    cmd_act_nn_ndx = knnsearch(FK_commands, FK_encoder);
    
    dev_command_estimate = ((FK_commands(cmd_est_nn_ndx,1)-FK_stepgen(:,1)).^2+...
        (FK_commands(cmd_est_nn_ndx,2)-FK_stepgen(:,2)).^2+...
        (FK_commands(cmd_est_nn_ndx,3)-FK_stepgen(:,3)).^2).^.5;
    [max_d_cmd_est, ndx_cmd_est] = sort(dev_command_estimate, 'descend');
    [max_d_cmd_est_path, ndx_cmd_est_path] = sort(dev_command_estimate(reorient_end_indices(pass):end), 'descend');
    
	dev_command_actual = ((FK_commands(cmd_act_nn_ndx,1)-FK_encoder(:,1)).^2+...
        (FK_commands(cmd_act_nn_ndx,2)-FK_encoder(:,2)).^2+...
        (FK_commands(cmd_act_nn_ndx,3)-FK_encoder(:,3)).^2).^.5;
    [max_d_cmd_act, ndx_cmd_act] = sort(dev_command_actual, 'descend');
    
    dev_estimate_actual = ((FK_stepgen(:,1)-FK_interp_encoder(:,1)).^2+...
        (FK_stepgen(:,2)-FK_interp_encoder(:,2)).^2+...
        (FK_stepgen(:,3)-FK_interp_encoder(:,3)).^2).^.5;
    [max_d_est_act, ndx_est_act] = sort(dev_estimate_actual, 'descend');
    
%     dev_command_estimate = ((FK_commands(:,1)-FK_interp_stepgen(:,1)).^2+...
%         (FK_commands(:,2)-FK_interp_stepgen(:,2)).^2+...
%         (FK_commands(:,3)-FK_interp_stepgen(:,3)).^2).^.5;
%     [max_d_cmd_est, ndx_cmd_est] = sort(dev_command_estimate, 'descend');
    
%     dev_command_actual = ((FK_commands(:,1)-FK_interp_encoder(:,1)).^2+...
%         (FK_commands(:,2)-FK_interp_encoder(:,2)).^2+...
%         (FK_commands(:,3)-FK_interp_encoder(:,3)).^2).^.5;
%     [max_d_cmd_act, ndx_cmd_act] = sort(dev_command_actual, 'descend');
    
%     dev_estimate_actual = ((FK_interp_stepgen(:,1)-FK_interp_encoder(:,1)).^2+...
%         (FK_interp_stepgen(:,2)-FK_interp_encoder(:,2)).^2+...
%         (FK_interp_stepgen(:,3)-FK_interp_encoder(:,3)).^2).^.5;
%     [max_d_est_act, ndx_est_act] = sort(dev_estimate_actual, 'descend');
    
%     figure(1);
%     plot3(FK_commands(:,1), FK_commands(:,2), FK_commands(:,3), 'b',...
%         FK_stepgen(:,1), FK_stepgen(:,2), FK_stepgen(:,3),'r--',...
%         FK_encoder(:,1), FK_encoder(:,2), FK_encoder(:,3), 'k');
    
    figure((pass-1)*2+1);
    hold on;
    plot3(FK_commands(:,1), FK_commands(:,2), FK_commands(:,3), 'b',...
        FK_stepgen(:,1), FK_stepgen(:,2), FK_stepgen(:,3),'r--',...
        FK_encoder(:,1), FK_encoder(:,2), FK_encoder(:,3), 'k');
%     plot3(FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),1), ...
%         FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),2), ...
%         FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),3),'o','MarkerFaceColor','g','MarkerSize',8,'MarkerEdgeColor','k','LineWidth',3);
    plot3(FK_stepgen(ndx_cmd_est_path(1:10*deviation_points(pass)),1), ...
        FK_stepgen(ndx_cmd_est_path(1:10*deviation_points(pass)),2), ...
        FK_stepgen(ndx_cmd_est_path(1:10*deviation_points(pass)),3),'o','MarkerFaceColor','g','MarkerSize',8,'MarkerEdgeColor','k','LineWidth',2);
	grid on;
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    %xlabel('X Tool Tip Position in WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    %ylabel('Y Tool Tip Position in WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    %zlabel('Z Tool Tip Position in WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
	xlabel('X WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    ylabel('Y WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    zlabel('Z WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    hold off;
    view(view_configs{pass}{1}(1), view_configs{pass}{1}(2));
    legend('Commanded Path','Estimated Path','Actual Path','Location','Best');
%     h = rotate3d; 
%     set(h, 'ActionPreCallback', 'set(gcf,''windowbuttonmotionfcn'',@align_axislabel)') 
%     set(h, 'ActionPostCallback', 'set(gcf,''windowbuttonmotionfcn'','''')') 
%     set(gcf, 'ResizeFcn', @align_axislabel) 
%     align_axislabel([], gca) 
    %axislabel_translation_slider;
    %view(46,22)
    %camzoom(4);
    %camtarget([FK_stepgen(ndx_cmd_est(1),1), FK_stepgen(ndx_cmd_est(1),2), FK_stepgen(ndx_cmd_est(1),3)])
    
    figure((pass-1)*2+2);
    hold on;
    plot3(FK_commands(:,1), FK_commands(:,2), FK_commands(:,3), 'b',...
        FK_stepgen(:,1), FK_stepgen(:,2), FK_stepgen(:,3),'r--',...
        FK_encoder(:,1), FK_encoder(:,2), FK_encoder(:,3), 'k');
%     plot3(FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),1),...
%         FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),2), ...
%         FK_stepgen(ndx_cmd_est(deviation_start_points(pass):deviation_start_points(pass)+deviation_points(pass)),3),'o','MarkerFaceColor','g','MarkerSize',8,'MarkerEdgeColor','k','LineWidth',3);
    plot3(FK_interp_encoder(ndx_cmd_est_path(1:deviation_points(pass)),1), ...
        FK_interp_encoder(ndx_cmd_est_path(1:deviation_points(pass)),2), ...
        FK_interp_encoder(ndx_cmd_est_path(1:deviation_points(pass)),3),'o','MarkerFaceColor','g','MarkerSize',8,'MarkerEdgeColor','k','LineWidth',1);
	grid on;
    ax = gca; ax.FontName = 'Times'; ax.FontSize = 12;
    xlabel('X WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    ylabel('Y WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    zlabel('Z WPC (mm)', 'FontName', 'Times', 'FontSize', 12);
    hold off;
    view(view_configs{pass}{2}(1), view_configs{pass}{2}(2))
    camzoom(zooms(pass));
    camtarget([FK_interp_encoder(ndx_cmd_est_path(deviation_start_points(pass)),1), ...
        FK_interp_encoder(ndx_cmd_est_path(deviation_start_points(pass)),2), ...
        FK_interp_encoder(ndx_cmd_est_path(deviation_start_points(pass)),3)])
    legend('Commanded Path','Estimated Path','Actual Path','Location','Best');
    
    if export_figures
        fig_heights = [4 7];
        for k = 1:2
            figure((pass-1)*2+k)
            set(gcf, 'PaperUnits', 'inches');
            set(gcf, 'PaperSize', [6 fig_heights(k)]);
            set(gcf, 'PaperPositionMode', 'manual');
            set(gcf, 'PaperPosition', [0 0 6 fig_heights(k)]);
            print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' pass_names{pass} '_3D' wpc_subnames{k}],'-dpdf', '-r500');
        end
    end
    
    fprintf('For Pass %i, the MIN deviation between command and estimate is %d\n', pass, max_d_cmd_est(end));
    fprintf('For Pass %i, the MEAN deviation between command and estimate is %d\n', pass, mean(max_d_cmd_est));
    fprintf('For Pass %i, the MAX deviation between command and estimate is %d\n', pass, max_d_cmd_est(1));
    fprintf('For Pass %i, the STD deviation between command and estimate is %d\n', pass, std(max_d_cmd_est));
    
    fprintf('For Pass %i, the MIN deviation between command and actual is %d\n', pass, max_d_cmd_act(end));
    fprintf('For Pass %i, the MEAN deviation between command and actual is %d\n', pass, mean(max_d_cmd_act));
    fprintf('For Pass %i, the MAX deviation between command and actual is %d\n', pass, max_d_cmd_act(1));
    fprintf('For Pass %i, the STD deviation between command and actual is %d\n', pass, std(max_d_cmd_act));
    
    fprintf('For Pass %i, the MIN deviation between estimate and actual is %d\n', pass, max_d_est_act(end));
    fprintf('For Pass %i, the MEAN deviation between estimate and actual is %d\n', pass, mean(max_d_est_act));
    fprintf('For Pass %i, the MAX deviation between estimate and actual is %d\n', pass, max_d_est_act(1));
    fprintf('For Pass %i, the STD deviation between estimate and actual is %d\n', pass, std(max_d_est_act));
    %mean(max_d_cmd_act)
    %mean(max_d_est_act)
    
end

%%
% G-Code Analysis
headtop_points = dlmread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Analyze Path Points\head_top.txt',' ');
headtop_points = headtop_points(2:end,:);
headbot_points = dlmread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Analyze Path Points\head_bottom.txt',' ');
headbot_points = headbot_points(2:end,:);
choldertop_points = dlmread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Analyze Path Points\cholder_top.txt',' ');
choldertop_points = choldertop_points(2:end,:);
cholderbot_points = dlmread('C:\Users\robyl_000\Documents\Projects\PocketNC\Experimental Data\Analyze Path Points\cholder_bottom.txt',' ');
cholderbot_points = cholderbot_points(2:end,:);

close all;
direct_times = [492.471 465.630 1042.701 158.788];
gcode_times1 = [534.365 525.995 1304.87349 216.97265];
gcode_times2 = [534.381 525.9347 1304.9574 217.0677];
avg_gcode_times = (gcode_times1+gcode_times2)/2;

path_distance = [102.9185 104.9013 218.8575 42.116];
path_distance_no_ret = [40.23357 44.32076 103.5884 23.57325];
distance_ratio = path_distance_no_ret./path_distance;
voxel_points = [24401 28669 59678 12603];
cutting_points = [13578 14816 35535 7248];
volume_removed_points = [length(headtop_points(headtop_points(:,7)~=0,:)), ...
    length(headbot_points(headbot_points(:,7)~=0,:)), ...
    length(choldertop_points(find(choldertop_points(:,7)~=0 & choldertop_points(:,6)==1),:))+length(choldertop_points(choldertop_points(:,6)~=1)), ...
    length(cholderbot_points(cholderbot_points(:,7)~=0,:))]
rets = [10 11 21 6];
gains = (avg_gcode_times - direct_times)./avg_gcode_times*100;
%gains(end) = .2682

k = cutting_points./path_distance;
%hold on;
%plot(k,gains,'b*-')
%plot(volume_removed_points./path_distance,gains,'b-*');
plot(distance_ratio,gains,'b-*');
%hold off;
ax = gca; ax.FontName = 'Arial'; ax.FontSize = 12;
xlabel('{\it R}_{PL}', 'Interpreter', 'tex', 'FontName', 'Arial', 'FontSize', 12);
ylabel('Reduction in Machining Time (%)', 'FontName', 'Arial', 'FontSize', 12);

set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [6 4]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 6 4]);
print(['C:\Users\robyl_000\Documents\Dissertation\Figures\Analysis Plots\' 'GCode_comparison_arial'],'-dpdf', '-r500');