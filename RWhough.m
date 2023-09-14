function [C,ThAxis,RAxis]=RWhough(Phi,D,NTh,Delta);
% function [C,ThAxis,RAxis]=RWhough(Phi,D,NTh,Delta);
%
% Calculate the Hough transform of the points given in D, Phi
%
% Input:
%  D     - A 1*N vector of distances to edge points.
%  Phi   - A 1*N vector of directions to edge points.
%  NTh   - The number of angular steps (angular resolution=360/NTh degrees)
%          Should be an even number.
%  Delta - The distance resolution
%
% Output:
%  C     - The Hough transform. A (max(D)/Delta)*NTh matrix.
%  ThAxis- The angles. A 1*NTh vector.
%  RAxis - The distances. A 1*(max(D)/Delta) vector.
%  

% Setup:

dTheta=2*pi/NTh; 
DMax=max(D);
NrOfMeas=length(D);
Nr=floor(DMax/Delta+1.5);        % Number of distance steps.
C=zeros(Nr,NTh);

ThAxis=dTheta*(0:(NTh-1)); 
RAxis=Delta*(1:Nr);

% Loop over all possible theta angles:
tres=0.1;
for i=1:(NTh/2);
  Theta=ThAxis(i);

  R=D.*cos(Phi-Theta);
  for j=1:length(R);
    if R(j)>0;
      Ri=floor(1.5+R(j)/Delta);
      C(Ri,i)=C(Ri,i)+R(j); %(abs(R(j))<tres)*R(j);
    else 
      Ri=floor(1.5-R(j)/Delta);
      C(Ri,i+NTh/2)=C(Ri,i+NTh/2)-R(j); %(abs(R(j))<tres)*R(j);
    end;
  end;
 % fprintf('Done with angle %g of %g\n',Theta,pi);
end;


