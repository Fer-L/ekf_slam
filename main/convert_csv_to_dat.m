data = readmatrix('odom_data.csv');
writematrix(data,'odom_data.dat','Delimiter',' ');
fid = fopen('odom_data.dat');
dat_data = fread(fid,'*char').';
fclose(fid);
disp(dat_data);