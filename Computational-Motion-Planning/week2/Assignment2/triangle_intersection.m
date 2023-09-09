function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
    flag = true;
    for i=1:3
        for j=1:3
            if(i==j)
                continue
            end
            [a,b,c] = TwoPoints2Line(P1(i,:), P1(j,:));
            p1 = P1(6-i-j, :);
            side1 = LinePointSide(a,b,c,p1);
            counter = 0;
            for k=1:3
                side2 = LinePointSide(a,b,c,P2(k,:));
                if(side1 ~= side2 && side2 ~= 0)
                    counter = counter + 1;
                end
            end
            if (counter == 3)
                flag = false;
            end
        end
    end

    for i=1:3
        for j=1:3
            if(i==j)
                continue
            end
            [a,b,c] = TwoPoints2Line(P2(i,:), P2(j,:));
            p1 = P2(6-i-j, :);
            side1 = LinePointSide(a,b,c,p1);
            counter = 0;
            for k=1:3
                side2 = LinePointSide(a,b,c,P1(k,:));
                if(side1 ~= side2 && side2 ~= 0)
                    counter = counter + 1;
                end
            end
            if (counter == 3)
                flag = false;
            end
        end
    end

    function [a,b,c] = TwoPoints2Line(p1, p2)
        b = p2(1) - p1(1);
        a = -(p2(2) - p1(2));
        c = -a*p1(1) -b*p1(2);
    end

    function side = LinePointSide(a,b,c,p)
        side = sign(a*p(1) + b*p(2) + c);
    end
end
% *******************************************************************
