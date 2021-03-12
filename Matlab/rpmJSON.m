function json = rpmJSON(left, right)
s = struct('type',"rpm",'rpm_l',left,'rpm_r',right);
json = jsonencode(s);
json = double(json);
json = [json,42];
end
