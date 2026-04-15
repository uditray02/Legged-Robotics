function projectile_animation(t_all, q_all, projectile)

% Create fixed-size figure and axes (CRITICAL)
fig = figure('Position',[100 100 600 600]);
ax = axes('Parent',fig);
set(ax,'Units','pixels','Position',[50 50 500 500]);

%%% interpolate for smooth animation %%
tstart = t_all(1); 
tend = t_all(end);
tinterp = linspace(tstart, tend, projectile.movieFps*(tend-tstart));

[m,n] = size(q_all);
qinterp = zeros(length(tinterp), n);

for i = 1:n
    qinterp(:,i) = interp1(t_all, q_all(:,i), tinterp);
end

% FIXED AXIS LIMITS (compute once)
xmin = -1;
xmax = max(qinterp(:,1)) + 1;
ymin = min(qinterp(:,3)) - 1;
ymax = max(qinterp(:,3)) + 1;

axis(ax, [xmin xmax ymin ymax]);
axis(ax, 'manual');   % lock axis
grid(ax, 'on');

% Video setup
if projectile.movieWrite
    mov = VideoWriter(projectile.movieName);
    mov.FrameRate = projectile.movieFps;
    open(mov);
end

%%% animation loop %%
for i = 1:length(tinterp)
    
    % Plot current point
    plot(ax, qinterp(i,1), qinterp(i,3), 'ro', ...
        'MarkerFaceColor','r','MarkerSize',10);
    hold(ax,'on');
    
    % Plot trajectory
    plot(ax, qinterp(1:i,1), qinterp(1:i,3), 'k','LineWidth',1.5);
    
    xlabel(ax,'x');
    ylabel(ax,'y');
    
    % Capture frame (ONLY axes → fixed size)
    if projectile.movieWrite
        frame = getframe(ax);
        writeVideo(mov, frame);
    end
    
    pause(projectile.moviePause);
    hold(ax,'off');
end

% Close video file
if projectile.movieWrite
    close(mov);
end

end
