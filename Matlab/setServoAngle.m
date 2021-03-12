function json = setServoAngle(ard, angle)
s = struct('type',"servo",'angle',angle);
json = jsonencode(s)
json = double(json);
json = [json,42];
write(ard,json)
end