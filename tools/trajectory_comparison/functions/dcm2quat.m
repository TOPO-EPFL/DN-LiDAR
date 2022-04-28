function attQ = dcm2quat( attC )

n = round(length(attC)/3);
c = 3*n;

attQ = zeros(4,n);

aux = 1 + diag(attC(1:3:c,1:3:c) + attC(2:3:c,2:3:c) + attC(3:3:c,3:3:c));
aux = aux';

if ~isempty(aux(aux < 0))
    return;
end

attQ(1,:) = sqrt(aux)./2.0;
d = ( 4.*attQ(1,:) );

attQ(2,:) = ( diag(attC(3:3:c,2:3:c) - attC(2:3:c,3:3:c)) )' ./ d;
attQ(3,:) = ( diag(attC(1:3:c,3:3:c) - attC(3:3:c,1:3:c)) )' ./ d;
attQ(4,:) = ( diag(attC(2:3:c,1:3:c) - attC(1:3:c,2:3:c)) )' ./ d;

return;