serialPort = 'COM3';            % define COM port #
plotTitle = 'Serial Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -1.5;                     % set y-min
max = 1.5                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution
 
%Define Function Variables
time = 0;
data = 0;
data2 = 0;
data3 = 0;
data4 = 0;
data5 = 0;
data6 = 0;
data7 = 0;
count = 0;
 
%Set up Plot
plotGraph = plot(time,data,time,data2,time,data3,time,data4);
             
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);
 
%Open Serial COM Port
s = serial('COM3', 'BaudRate', 9600,'Parity', 'none', 'DataBits', 8, 'StopBit', 1,'TERMINATOR', 'LF')
disp('Close Plot to End Session');
fopen(s);
 
tic
 
while ishandle(plotGraph) %Loop when Plot is Active
     
     
	 dat = fscanf(s,'%f');
  
   
     %Read Data from Serial as Float
  
    if(~isempty(dat)) %Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;    %Extract Elapsed Time
		data(count) = dat(1,1);
        data2(count) = dat(2,1); %Extract 1st Data Element         
          data3(count) = dat(3,1); 
		  data4(count) = dat(4,1); 
		  
		  
        %Set Axis according to Scroll Width
		
       hold on
plot(time,data,'r','LineWidth',2)
plot(time,data2,'b','LineWidth',2)
plot(time,data3,'g','LineWidth',2)
plot(time,data4,'k','LineWidth',2)



        
        
        
		if(scrollWidth > 0)
        set(plotGraph,'XData',time(time > time(count)-scrollWidth),'YData',data(time > time(count)-scrollWidth));
        axis([time(count)-scrollWidth time(count) min max]);
        else
        set(plotGraph,'XData',time,'YData',data);
        axis([0 time(count) min max]);
        end
		hold off
        %Allow MATLAB to Update Plot
        pause(delay);
    end
end
 
%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;
 fclose(instrfind)

 
disp('Session Terminated...');