%% 3d visualization
figure;
ax = axes('XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[-1.5 1.5]);
set(gca,'XTick',[],'YTick',[], 'zTick',[])
view(3); grid on; axis equal; hold on
xlabel('X','FontSize',20,'FontWeight','bold'); ylabel('Y','FontSize',20,'FontWeight','bold'); zlabel('Z','FontSize',20,'FontWeight','bold');

% create the surface
len = 1;
wid = 0.5;
[X, Y, Z] = cylinder([.05 .05]);
h(1) = surface(wid*Z-wid/2, X-len/2, Y, 'FaceColor','blue');
h(2) = surface(-wid*Z+wid/2, X+len/2, Y,'FaceColor','cyan');
h(3) = surface(Y-wid/2, len*Z-len/2, X, 'FaceColor','magenta');
h(4) = surface(Y+wid/2, -len*Z+len/2, X, 'FaceColor','yellow');

% create hgtransform object
t = hgtransform('Parent',ax);
set(h,'Parent',t)
set(gcf,'Renderer','opengl')
drawnow
Rz = eye(4);
Sxy = Rz;

for ii = 1:floor(nsteps/10)
    % Z-axis rotation matrix 
    
    Rz = makehgtform('zrotate',posterior_euler(ii*10,1));
    Ry = makehgtform('yrotate',posterior_euler(ii*10,2));
    Rx = makehgtform('xrotate',posterior_euler(ii*10,3));

    % set the hgtransform Matrix property
    set(t,'Matrix',Rx*Ry*Rz)
    drawnow
    
    % frames
    frames(ii) = getframe(gcf);
    pause(0.01);
end

% save video
video = VideoWriter('mekf_accel_gyro_mag','MPEG-4');
video.FrameRate = 10;
open(video)
writeVideo(video,frames);
close(video)