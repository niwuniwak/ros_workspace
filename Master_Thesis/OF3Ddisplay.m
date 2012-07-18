clc 
clear all
%% OF 3D loading file
tic
%fid = fopen('myfile.bin', 'r');
%[data, count] = fread(fid, 'int');
data = load('myfile.txt');
count = length(data);
%fclose(fid);
toc

prevx = data(1:6:count-5);
prevy = data(2:6:count-4);
prevz = data(3:6:count-3);
currx = data(4:6:count-2);
curry = data(5:6:count-1);
currz = data(6:6:count);



% for j=1:length(prevx)
%     hold on;
%     plot3([prevx(i) currx(i)], [prevy(i) curry(i)], [prevz(i) currz(i)]);
% end
X3D = [prevx currx];
Y3D = [prevy curry];
Z3D = [prevz currz];
%% Display
XMIN = 0;
XMAX = 480;
YMIN = 0;
YMAX = 2000;
ZMIN = 0;
ZMAX = 640;

toc
f=0;

fig = figure(1);


%axis zx;
view(-45, 30);

frame_nb=round(count/6/307200)

aviobj = avifile('mymovie.avi','compression','None');

for i=0:frame_nb-1
    axis on;
    %axis vis3d;
    hold off;
    for j=1:50:307200
        xlabel('x');
        ylabel('z');
        zlabel('y');
        axis([XMIN XMAX YMIN YMAX ZMIN ZMAX]);
        if (prevx(j+i*307200) ~= currx(j+i*307200)) && (prevy(j+i*307200) ~= curry(j+i*307200) && (prevz(j+i*307200) ~=0))
            f=f+1;
            plot3(X3D(j+i*307200,:), Z3D(j+i*307200,:), Y3D(j+i*307200,:));
            hold on;
        end
    end
    F = getframe(fig);
    aviobj = addframe(aviobj,F);
    f=0; 
    toc
    pause;
    hold off;
end

aviobj = close(aviobj);
