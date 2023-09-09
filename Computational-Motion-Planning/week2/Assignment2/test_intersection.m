x = [2,2,3]; 
y = [5,3,4];
P1 = [x;y]';

x = [0,4,2];
y = [0,4,6];
P2 = [x;y]';

x = [1,1,4]; 
y = [1,3,2];
P1 = [x;y]';

x = [4,4,5];
y = [1.5,2.5,2];
P2 = [x;y]';

x = [1,1,4]; 
y = [1,3,2];
P1 = [x;y]';

x = [4,4,5];
y = [1,1.9,1.5];
P2 = [x;y]';



line([P1(:,1)' P1(1,1)],[P1(:,2)' P1(1,2)],'Color','r')
line([P2(:,1)' P2(1,1)],[P2(:,2)' P2(1,2)],'Color','b')

flag = triangle_intersection(P1,P2);
if flag == 1
    disp("Intersection")
else
    disp("No Intersection")
end