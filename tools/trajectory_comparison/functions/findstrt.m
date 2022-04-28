function [si, ei]= findstrt(input,startime,endtime)
% finds startime and entime index
% if the startime endtime is out of bound, returns si=0 ei=length(input)

first = input(1);
last = input(end);

if(endtime<startime),
   temp = endtime; 
   endtime = startime;
   startime=temp; 
end

if startime<first,
   si = 1; 
elseif startime>last,
   si = length(input);
else
   si_all =find(input(:)>=startime);
   si = si_all(1);
end

if endtime <first,
   ei = 1;
elseif endtime>last,
   ei = length(input);
else
   ei_all = find(input>=endtime);
   ei = ei_all(1);
end
