m=1;
n=length(position(1,:));
while (m< (n+4)/6)
    [X1]=forward_kin(position(1,6*m-4),position(2,6*m-4),position(3,6*m-4));
    [X2]=forward_kin(position(1,6*m-3),position(2,6*m-3),position(3,6*m-3));
    [X3]=forward_kin(position(1,6*m-2),position(2,6*m-2),position(3,6*m-2));
    [X4]=forward_kin(position(1,6*m-1),position(2,6*m-1),position(3,6*m-1));
    [angle1]=inverse_kin(X1(1),X1(2),X1(3));
    [angle2]=inverse_kin(X2(1),X2(2),X2(3));
    [angle3]=inverse_kin(X3(1),X3(2),X3(3));
    [angle4]=inverse_kin(X4(1),X4(2),X4(3));
    for i=4:9
        position(i,6*m-4)=angle1(i);
        position(i,6*m-3)=angle2(i);
        position(i,6*m-2)=angle3(i);
        position(i,6*m-1)=angle4(i);
    end
    m=m+1;
end