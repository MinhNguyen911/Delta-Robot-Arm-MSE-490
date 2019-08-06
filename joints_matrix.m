for i=2:35
    for m=1:9
        if(joints2(m,i)<-90 && abs(joints_2(m,i)-joints_2(m,i-1))>=180)
            joints2(m,i)=joints2(m,i)+360;
        end
    end
end
