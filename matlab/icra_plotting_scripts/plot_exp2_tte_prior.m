clear all
clf
close all

%% Load data!
load('exp_2_tte_as_fun_of_prior_large.mat')

%% Setup data.
belief_low_conf_init = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
beliefs = cell2mat(belief_low_conf_init);
hold on
all_init_conds = all_ttes_per_init_cond.keys();
rc = linspace(0.3,0.3,length(all_ttes_per_init_cond));
bc = linspace(0.4,0.9,length(all_ttes_per_init_cond));

sum_per_prior = zeros(1,length(belief_low_conf_init));
tte_per_prior = zeros([length(belief_low_conf_init), length(all_ttes_per_init_cond)]);
for i=1:length(all_ttes_per_init_cond)
    init = all_init_conds{i};
    all_ttes = all_ttes_per_init_cond(init);
    all_ttes = [cell2mat(all_ttes), 0];
    tte_per_prior(:,i) = all_ttes';
    %% Plot!
    %plot(beliefs, all_ttes, 'o', 'linewidth', 2, ...
    %    'color', [1,1-rc(i),1-rc(i)], ...
    %    'markerfacecolor', 'none'); %[1,1-rc(i),1-rc(i)]);
    sum_per_prior = sum_per_prior + all_ttes;
end

alpha = 0.5;
acolor = 'r';
[lineOut, fillOut] = stdshade(tte_per_prior', alpha,acolor);
lineOut.Marker = 'o';
lineOut.MarkerFaceColor = 'r';
lineOut.MarkerSize = 5;

%plot(beliefs, sum_per_prior/length(all_ttes_per_init_cond), 'o-', 'linewidth', 2, ...
%    'color', 'r', ...
%    'markerfacecolor', 'r');

xlim([1,9])
%ylim([0,4])
ylabel('$TTL$', 'Interpreter', 'latex', 'fontsize', 25)
xlabel('$b^0(\beta = 0)$', 'Interpreter', 'latex', 'fontsize', 25)
title('TTL as function of prior', 'Interpreter', 'latex', 'fontsize', 24)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,400,700])
xticks(linspace(1,9,9))
xticklabels({'0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9'})
yticks([0,0.5,1,1.5,2,2.5,3])
box off
grid off