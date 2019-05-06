t=xAngleValue;
r=ones(size(t,1),1);
t=t';
r=r';
[x,y]=pol2cart(t,r);
compass(x,y)