bag=csv2mtlb('process.txt');
aux=ones(size(bag.x,1),1);
data=[bag.x-aux*bag.x(1), bag.y-aux*bag.y(1), bag.z-aux*bag.z(1)];
covariance=cov(data)