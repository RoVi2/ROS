bag=csv2mtlb('process.txt');
td(1)=0;
for i=2:1:size(bag.time)
    td(i)=(bag.time(i)-bag.time(i-1))/(10^9);
end
speed_x(1)=0;
speed_y(1)=0;
speed_z(1)=0;
for i=2:1:size(bag.time)
    speed_x(i)=(bag.x(i)-bag.x(i-1))/td(i);
    speed_y(i)=(bag.y(i)-bag.y(i-1))/td(i);
    speed_z(i)=(bag.z(i)-bag.z(i-1))/td(i);
end
acc_x(1)=0;
acc_y(1)=0;
acc_z(1)=0;
for i=2:1:size(bag.time)
    acc_x(i)=(speed_x(i)-speed_x(i-1))/td(i);
    acc_y(i)=(speed_y(i)-speed_y(i-1))/td(i);
    acc_z(i)=(speed_z(i)-speed_z(i-1))/td(i);
end

aux=ones(size(bag.x,1),1);
data=[bag.x-aux*bag.x(1) bag.y-aux*bag.y(1) bag.z-aux*bag.z(1) speed_x' speed_y' speed_z' acc_x' acc_y' acc_z'];
covariance=cov(data)