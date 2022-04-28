function compareTrajectories(config)

% load the stuff

global cache

for i = 1:length(config.trjs)
    
    fprintf('Reading trajectory %d ...\n', i);
    
    file = dir(config.trjs{i}.path);
    if isempty(file)        
        error(sprintf('file %s does not exist\n', config.trjs{i}.path));
    end
    
    in_cache = false;
    
    cur_date = file.datenum;
    
    for j = 1:length(cache)
       if strcmp(cache{j}.path, config.trjs{i}.path)
           
          if cache{j}.date == cur_date
              trjs{i} = cache{j}.trj;
              
              fprintf('Found in cache\n');
              in_cache = true;
          end              
       end
    end
    
    if in_cache ~= true
        switch config.trjs{i}.type
            case 'roamfree'
                trjs{i} = loadTrajectory_roamfree(config.trjs{i});

            case 'posproc'
                trjs{i} = loadTrajectory_posproc(config.trjs{i});

        end
        
        cache{end+1} = struct('path', config.trjs{i}.path, 'date', cur_date, 'trj', trjs{i});
        
        fprintf('Done\n');
    end
end

% plot

close all

rpy_titles = {'Roll', 'Pitch', 'Yaw'};
enu_titles = {'East', 'North', 'Up'};

if isfield(config, 'diffwrtref') && config.diffwrtref == true
    i = setdiff(1:length(trjs),[config.reference]);
    
    figure
    for j = i
        %rpy_int = interp1(trjs{j}.t, trjs{j}.rpy, trjs{config.reference}.t);        
        q_int = interp1(trjs{j}.t, trjs{j}.q, trjs{config.reference}.t);
        
        q_diff = zeros(size(q_int,1), 4);
        for h = 1:size(q_int,1)
           q_diff(h,:) = quatprod(quatinv(quatnorm(q_int(h,:))), trjs{config.reference}.q(h,:)); 
        end
        err_rpy = quat2euler(q_diff);
        
        for ax = 1:3
            err_rpy(:,ax) = rad2deg(unwrap(err_rpy(:, ax)));
            
            subplot(1,3,ax)
            hold on
            
            %plot(rad2deg( angdiff(trjs{config.reference}.rpy(:,ax), rpy_int(:,ax)) ));
            hold on
            plot(err_rpy(:,ax));
            
            % linear interpolation
            coeffs = polyfit(trjs{config.reference}.t, err_rpy(:,ax),1);
            plot(coeffs(1)*trjs{config.reference}.t+coeffs(2),'lineWidth',2);
            
            grid on
            axis tight
            
            title(rpy_titles{ax})
            ylabel('[deg]');
            xlabel('time [s]');
        end
    end

    figure
    for j = i
        enu_int = interp1(trjs{j}.t, trjs{j}.enu, trjs{config.reference}.t);
        
        for ax = 1:3
            subplot(1,3,ax)
            hold on
            
            plot(trjs{config.reference}.enu(:,ax) - enu_int(:,ax));
            
            grid on
            axis tight
            
            title(enu_titles{ax})
            ylabel('[m]');
            xlabel('time [s]');
        end
    end
    
    
else
    figure
    for ax = 1:3
        subplot(1,3,ax)
        hold on

        for i = 1:length(trjs)
            plot(trjs{i}.t, rad2deg(trjs{i}.rpy(:,ax)) )
        end    

        grid on
        axis tight
        
        title(rpy_titles{ax});
        ylabel('[deg]');
        xlabel('time [s]');
    end
    
    figure
    for ax = 1:3
        subplot(1,3,ax)
        hold on

        for i = 1:length(trjs)
            plot(trjs{i}.t, trjs{i}.enu(:,ax))
        end    

        grid on
        axis tight
        
        title(enu_titles{ax});
        ylabel('[m]');
        xlabel('time [s]');
    end
    
end

end

function trj = loadTrajectory_roamfree(cfg)

    if strcmp(cfg.path(end-5:end),'tar.gz')

        j = find(cfg.path == '/', 1, 'last');
        tmp_path = ['/tmp/' cfg.path(j+1:end-7)];

        if exist(tmp_path, 'file') == 7  
            rmdir(tmp_path, 's');
        end

        mkdir(tmp_path);
        system(['tar xzf ''' cfg.path ''' -C ''' tmp_path '''']);

        cfg.path = tmp_path;    
    end

    raw = load([cfg.path '/PoseSE3(W).log']);

    [~, j] = sort(raw(:,1));

    trj.t = raw(j,1);
    trj.enu = raw(j,3:5);
    trj.q = raw(j,6:9);
    
    if isfield(cfg, 'Qbs')
        for i = 1:length(trj.t)
            trj.q(i,:) = quatprod(trj.q(i,:), cfg.Qbs);
        end
    end
    
    trj.rpy = quat2euler(trj.q);
    
%     lla = ENUtolatlonwithgtran(trj.enu, cfg.lat0, cfg.lon0);    
%     trj.lv95 = latlonToCHwithgtran(lla);
end

function trj = loadTrajectory_posproc(cfg)

    psp = readPosprocFile(cfg.path, 'Format', 'SNV')';
    
    % this is if we have the time in SOW
    % trj.t = mod(psp(:,1), 24*60*60);
    trj.t = psp(:,1);
    
    rpy_orig = psp(:,8:10);
    
    % construct rotation matrix
    
    R_enu_ned = [0 1 0; 1 0 0; 0 0 -1]; %the same as T_ned_enu
    
    trj.q = zeros(length(trj.t), 4);    
    for i = 1:length(trj.t)
        R_b_ned = (R1(rpy_orig(i,1)) * R2(rpy_orig(i,2)) * R3(rpy_orig(i,3)));
        
        trj.q(i,:) = dcm2quat(R_enu_ned*R_b_ned');
    end
    
    trj.rpy = quat2euler(trj.q);
    
    %trj.lv95 = latlonToCHwithgtran([rad2deg(psp(:,2:3)) psp(:,4)]);
    
    ecef = lla2ecef([rad2deg(psp(:,2:3)) psp(:,4)], 'WGS84');    
    
%     trj.enu = latlonToENUwithgtran([rad2deg(psp(:,2:3)) psp(:,4)], cfg.lat0, cfg.lon0);

    [E,N,U] = ecef2enu(ecef(:,1), ecef(:,2), ecef(:,3), cfg.lat0, cfg.lon0, 0, wgs84Ellipsoid('meter'));  
    trj.enu = [E N U];
end


