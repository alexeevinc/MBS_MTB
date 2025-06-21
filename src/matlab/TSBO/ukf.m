% pyenv('Version','C:\Users\Serhii Alieksieiev\AppData\Local\Programs\Python\Python310\python.exe');
% folder_of_ukf = 'C:\1_A\TUM\MA\ukf\';
% cd(folder_of_ukf)
% parameter = [0.003, 50, 0.00015, 0.005];
% RMSE = py.main.main(parameter);

function y = ukf(x)
a = x;
pyenv('Version','C:\Users\Serhii Alieksieiev\AppData\Local\Programs\Python\Python310\python.exe');

folder_of_ukf = 'C:\1_A\TUM\MA\ukf\';

cd(folder_of_ukf)
cost = py.main_as_function.main(x);

y = cost;






