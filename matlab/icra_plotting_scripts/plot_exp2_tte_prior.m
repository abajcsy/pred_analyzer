clear all
clf
close all

%% Load data!
load('tte_as_fun_of_prior.mat')

%% Setup data.
belief_low_conf_init = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
beliefs = cell2mat(belief_low_conf_init);
hold on
all_init_conds = all_ttes_per_init_cond.keys();
rc = linspace(0.3,0.3,length(all_ttes_per_init_cond));
bc = linspace(0.4,0.9,length(all_ttes_per_init_cond));

sum_per_prior = zeros(1,length(belief_low_conf_init));

for i=1:length(all_ttes_per_init_cond)
    init = all_init_conds{i};
    all_ttes = all_ttes_per_init_cond(init);
    all_ttes = [cell2mat(all_ttes), 0];
    %% Plot!
    plot(beliefs, all_ttes, 'o', 'linewidth', 2, ...
        'color', [1,1-rc(i),1-rc(i)], ...
        'markerfacecolor', 'none'); %[1,1-rc(i),1-rc(i)]);
    sum_per_prior = sum_per_prior + all_ttes;
end

plot(beliefs, sum_per_prior/length(belief_low_conf_init), 'o-', 'linewidth', 2, ...
    'color', 'r', ...
    'markerfacecolor', 'r');

xlim([0.1,0.9])
ylim([0,4])
ylabel('$min~TTE$', 'Interpreter', 'latex', 'fontsize', 16)
xlabel('$b_0(\beta = unmodelled)$', 'Interpreter', 'latex', 'fontsize', 16)
title('min~TTE as function of prior', 'Interpreter', 'latex', 'fontsize', 20)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,500,800])
xticks(linspace(0.1,0.9,9))
yticks(linspace(0,4,5))
box on
grid off