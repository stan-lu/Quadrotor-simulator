function value=constrain(val,min,max)
if val>max
    value=max;
elseif val<min
    value=min;
else
    value=val;
end