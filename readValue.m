%fileName = input('Insert file name: ','s')
fileName = 'data';
values = readtable(fileName);
qtbl = [table2array(values(:,4)),table2array(values(:,5)),table2array(values(:,6)),table2array(values(:,7))];


%%
num = 0 
newy = zeros(1,length(y-1));
limit = 10*ones(1,length(y));
for n = 2:length(y)
    if (y(n)-y(n-1))>90
        newy(n-1)=y(n-1)+num;
        num = num-360;
    elseif (y(n)-y(n-1))<-90
        newy(n-1)=y(n-1)+num;
        num = num+360;
    else
        newy(n-1)=y(n-1)+num;
    end
end
% deal with last value here.
newy(length(y)) = y(length(y))+num;
figure
hold on
plot(newy)
plot(y)
plot(limit)
plot(-limit)
