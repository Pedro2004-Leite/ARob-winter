clear all
close all

Struct05 = load('height_kp05.mat');
Struct05.P = 0.5;
Struct05 = rmfield(Struct05,'tf1');
Struct05.step_kp = Struct05.step_kp_05;
Struct05.step_data = Struct05.stepdata;

Struct1 = load('height_kp1.mat');
Struct1.P = 1;
Struct1.step_kp = Struct1.step_kp_1;

Struct2 = load('height_kp2.mat');
Struct2.P = 2;
Struct2.step_kp = Struct2.step_kp_2;

Struct3 = load('height_kp3.mat');
Struct3.P = 3;
Struct3.step_kp = Struct3.step_height_kp3;

cz = compareOptions('InitialCondition', 'z');
fig1 = figure();
Data = {Struct05, Struct1, Struct2, Struct3};
sgtitle('Test-Flight as Input')
for idx = 1:4
    Stru = Data{idx};
    subplot(2,2,idx) 
    Stru.com = compare(Stru.data,Stru.step_kp,cz);
    Stru.com.TimeUnit = '';
    %plot(com) 
    hold on
    plot(Stru.data.SamplingInstants,Stru.data.InputData)
    plot(Stru.data.SamplingInstants,Stru.data.OutputData)
    plot(Stru.com.SamplingInstants, Stru.com.OutputData)
    ylabel('height (m)')
    xlabel('Time (s)')
    legend off
    title('k_{P} = ' + string(Stru.P))
    grid on
end
saveas(fig1, 'Height_Controll_Response.png')
fig2 = figure();
sgtitle('Second Step as Input')
for idx = 1:4
    Stru = Data{idx};
    subplot(2,2,idx) 
    Stru.com = compare(Stru.step_data,Stru.step_kp,cz);
    Stru.com.TimeUnit = '';
    %plot(com) 
    hold on
    plot(Stru.step_data.SamplingInstants,Stru.step_data.InputData)
    plot(Stru.step_data.SamplingInstants,Stru.step_data.OutputData)
    plot(Stru.com.SamplingInstants, Stru.com.OutputData)
    ylabel('height (m)')
    xlabel('Time (s)')
    legend off
    title('k_{P} = ' + string(Stru.P))
    grid on
end
saveas(fig2, 'Height_Step_Response.png')