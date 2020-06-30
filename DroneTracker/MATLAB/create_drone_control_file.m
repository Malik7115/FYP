[n,m] = size(Ad);

f = fopen('../src/Thrust.cpp', 'w');

fprintf(f, '#include <opencv2/opencv.hpp>\n');
fprintf(f, '#include "DroneControl.hpp"\n');
fprintf(f, 'using namespace std;\n');
fprintf(f, 'using namespace cv; \n\n');


fprintf(f, 'double A1_data[%d][%d] = {', n, n);
for i= 1:n
    fprintf(f, '{');
    fprintf(f, '%.16g, ', Ad(i,:));
    fprintf(f, '},\n');
end
fprintf(f, '};\n');
fprintf(f,'Mat A1(%d, %d, CV_64F, A1_data);\n\n', n, n);

fprintf(f,'double B1_data[%d] = {', n);
fprintf(f, '%.16g, ', Bd);
fprintf(f, '};\n');
fprintf(f,'Mat B1(%d, 1, CV_64F, B1_data);\n\n', n);

fprintf(f,'double C1_data[%d] = {', n);
fprintf(f, '%.16g, ', Cd);
fprintf(f, '};\n');
fprintf(f,'Mat C1(1, %d, CV_64F, C1_data);\n\n', n);


fprintf(f,'double K1_data[%d] = {', n);
fprintf(f, '%.16g, ', Kd);
fprintf(f, '};\n');
fprintf(f,'Mat K1(1, %d, CV_64F, K1_data);\n\n', n);

% Observer 
F = Aobs-Hd*Cobs;
[n,m] = size(F);

fprintf(f,'double H1_data[%d] = {', n);
fprintf(f, '%.16g, ', Hd);
fprintf(f, '};\n');
fprintf(f,'Mat H1(%d, 1, CV_64F, H1_data);\n\n', n);

fprintf(f,'double G1_data[%d] = {', n);
fprintf(f, '%.16g, ', Bobs);
fprintf(f, '};\n');
fprintf(f,'Mat G1(%d, 1, CV_64F, G1_data);\n\n', n);

fprintf(f, 'double F1_data[%d][%d] = {', n, n);
for i= 1:n
    fprintf(f, '{');
    fprintf(f, '%.16g, ', F(i,:));
    fprintf(f, '},\n');
end
fprintf(f, '};\n');
fprintf(f,'Mat F1(%d, %d, CV_64F, F1_data);\n\n', n, n);


fprintf(f, 'observer thrust_observer(A1, F1, G1, H1, K1);\n\n');

fprintf(f, 'predictor thrust_pred;\n\n');

fprintf(f, 'predictor x_pred;\n\n');

fprintf(f, 'predictor z_pred;\n\n');



fclose(f);

