function stop(ard)
stop_cmd = '{"type":"rpm","rpm_l":0,"rpm_r":0}*';
stop_cmd = double(stop_cmd);
write(ard,stop_cmd);
end
