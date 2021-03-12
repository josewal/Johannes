clc, clear all

r = raspi("10.0.0.6", "pi", "raspberry")
pause(2);
ard = serialdev(r,'/dev/ttyACM0',115200)
pause(2);
cam = webcam(r)
pause(2);
display("CONNECTED")

h_fig = figure;
set(h_fig,'KeyPressFcn',@myfun);


while(1)
    frame = snapshot(cam);
    [BW,masked] = BLACKlineMask(frame);
        se = strel('disk',5);
        BW = imerode(BW,se);
        BW = imclose(BW,se);
        BW = imfill(BW,'holes');
    
    hold off
    %     imshowpair(frame,BW,'diff')
    imshow(masked);
    hold on
    
    [H,T,R] = hough(BW);
    P  = houghpeaks(H,20,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',150);
    
    if ~isempty(lines)
        endpoint_sum = 0;
        startpoint_sum = 0;
        
        for i = 1:length(lines)
            plot([lines(i).point1(1),lines(i).point2(1)],[lines(i).point1(2),lines(i).point2(2)], '-g')
            plot(lines(i).point1(1),lines(i).point1(2),'oy')
            endpoint_sum = endpoint_sum + lines(i).point1;
            startpoint_sum = startpoint_sum + lines(i).point2;
        end
        
        n = length(lines);
        
        endpoint = endpoint_sum/n;
        startpoint = startpoint_sum/n;
        midpoint = (startpoint + endpoint)/2;
        x = [startpoint(1), midpoint(1), endpoint(1)];
        y = [startpoint(2),midpoint(2), endpoint(2)];
        
        
        plot(x,y,'rx');
        
        
        
        start_offset = x(1) - 160;
        mid_offset = x(2) - 160;
        end_offset = x(3) - 160;
        
        u = [0,-32];
        v = [diff([x(1), x(2)]), diff([y(1), y(2)])];
        
        angle = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
        angle = acosd(angle);
        if endpoint(1) > 160
            angle = - angle;
        end
        
        
        
        title(['angle = ' num2str(angle)]);
        if angle > 5
            driveRPM(ard,-angle*1,angle*1)
        elseif angle < -5
            driveRPM(ard,-angle*1,angle*1)
        else
            driveRPM(ard,20,20)
        end
    else
        driveRPM(ard,-15,-15)
    end
    pause(0.1)
e
