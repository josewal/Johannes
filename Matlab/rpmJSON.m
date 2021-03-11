function json = rpmJSON(left, right, serv )
s = struct('rpm_l',left,'rpm_r',right);
json = jsonencode(s);
json = double(json);
json = [json,42];
end