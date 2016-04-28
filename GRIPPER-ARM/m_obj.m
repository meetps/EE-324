function m_obj
% Use serial data to communicate with Arduino and tell it to move arm in
% manner predetermined by image processing video from camera input.

% *************************************************************************
%    Set up Video input device
% *************************************************************************
    vid = videoinput('winvideo',1,'YUY2_640x480');
    triggerconfig(vid,'manual');
    set(vid,'FramesPerTrigger',1);
    set(vid,'TriggerRepeat', Inf);
    set(vid,'ReturnedColorSpace','rgb');

% *************************************************************************
%    Tweak your preferences
% *************************************************************************
    thresh_color1 = 16;
    thresh_color2 = 30;
    thresh_area1  = 30;
    thresh_area2  = 130;
    thresh_grab   = 99; 
    count_2       = 0;
    count_3       = 0;
    height        = 214/1.1;  
    side1         = 131/1.1;   
    side2         = 320/1.15;
    correction    = 40/1.1;
    
% *************************************************************************
%    Set up communication with arduino
% *************************************************************************
   
arduino=serial('COM14','BaudRate',9600); % create serial communication object on port COM4
fopen(arduino); % initiate arduino communication

% *************************************************************************
%    Start execution of program
% *************************************************************************
    
    start(vid); %Starts camera device
%     dist = zeros(1,10);
%     angle = zeros(1,10);
%     dist_avg = zeros(1,10);
%     angle_avg = zeros(1,10);
    
    
    % Change this loop structure according to your logic. You could keep a
    % for loop here for known number of iterations or end loop if you want
    % a flexible condition
       while (1)
        for count = 1:5
        trigger(vid);
        pause(.5)
        im = getdata(vid,1);
        figure(1);
        
        % imshow(im); % Shows image in a figure

        % Store separate RGB values in matrices
        r = im(:,:,1);
        g = im(:,:,2);
        b = im(:,:,3);

        % The given values are used to make a Green and a Red selection
        % respectively by ignoring other remaining colors.
        % Values may be changed according to environmental factors. If
        % extremely interested in how/why, please ask later.
        justSelection_g = g - r/2 - b/2;
        justSelection_r = r - g/2 - b/2;
        
        % Converting to binary black/white for clarity and decision making.
        black_white_g = justSelection_g > thresh_color1;
        black_white_r = justSelection_r > thresh_color2;
        imshow(black_white_g);
        figure(2);
        imshow(black_white_r);
        colormap('hot');
        
        % Removes smaller (noise) areas and concentrates on the major
        % object of choice only. To get good values, keep thresh larger;
        % but be careful.
        object_g_area = bwareaopen(black_white_g, thresh_area1);
        object_r_area = bwareaopen(black_white_r, thresh_area2);
        
        % Obtain centroid locations and respective areas of major object
        % groups of respective colors from image.
        stats_g = regionprops(object_g_area, {'centroid', 'area'});
        stats_r = regionprops(object_r_area, {'centroid', 'area'});
        
        [~, id_g] = max([stats_g.Area]);
        [~, id_r] = max([stats_r.Area]);
        
        if (id_g > 0) & (id_r > 0) % Please don't change & to && even though MATLAB tells you to
            
            % X and Y coordinates of centres 
            x_g = stats_g(id_g).Centroid(1);
            y_g = stats_g(id_g).Centroid(2);
            x_r = stats_r(id_r).Centroid(1);
            y_r = stats_r(id_r).Centroid(2);

            % Calculate distance between centres
            dist = sqrt((x_r-x_g)^2 + (y_r-y_g)^2);
                       
            % Uncomment this if you want to look at your centres
%              figure(3);
%              hold on
%              imshow(im);
%              plot([x_g, x_r], [y_g, y_r],'*')
%              text(540,400,num2str(dist));
%              if(cnt2==1)
%              text(140,100,num2str(dist_avg));
%              text(140,150,num2str(angle1));
%              text(140,200,num2str(angle2));
%              text(140,250,num2str(angle3));
%              end
%              hold off
        end
        
        if(count==1)
            x_red_mean=0;
            y_red_mean=0;
            x_green_mean=0;
            y_green_mean=0;
        end
        
        if(count>=5)
           x_red_mean   = (x_red_mean*(count-5)+x_r)/(count-4); 
           y_red_mean   = (y_red_mean*(count-5)+y_r)/(count-4); 
           x_green_mean = (x_green_mean*(count-5)+x_g)/(count-4); 
           y_green_mean = (y_green_mean*(count-5)+y_g)/(count-4);
        end
        
        % *****************************************************************
        %    Rest of your logic for giving control instructions to Arduino
        %    goes here.
        % *****************************************************************
        
        end
        
        mean_dist = sqrt((x_red_mean-x_green_mean)^2 + (y_red_mean-y_green_mean)^2);
        
        temp_thresh = (y_red_mean-y_green_mean)/(x_red_mean-x_green_mean);
        
        if (temp_thresh >=0)
                angle_avg = atand ((y_red_mean-y_green_mean)/(x_red_mean-x_green_mean));
        end
        
        if(temp_thresh < 0)
                angle_avg = 180 + atand ((y_red_mean-y_green_mean)/(x_red_mean-x_green_mean));
        end
    
    hypotenuse = sqrt((height)^2 + (mean_dist-correction)^2);
    alpha = ((hypotenuse)^2+(side1)^2-(side2)^2)/(2*hypotenuse*side1);
    beta = ((side1)^2+(side2)^2-(hypotenuse)^2)/(2*side1*side2);
    
    primaryOff = 90;
    secondaryOff = 75;
    
    theta1 = angle_avg+2;
    theta4 = acosd(alpha);
    theta5 = acosd(height/hypotenuse);
    theta2 = theta4+theta5-secondaryOff;
    theta3 = acosd(beta)-primaryOff;
   if(count_3==1)
            fprintf(arduino,'%s',char(theta1)); 
            fprintf(arduino,'%s',char(theta2));
            fprintf(arduino,'%s',char(theta3));
           fclose(arduino); % end communication with arduino
           break;
   end

    count2=1;
    count_3 = count_3+1;
       end
    % Necessary operation for closing program
    stop(imaqfind); 
end