%Example on the use of AStar Algorithm in an occupancy grid. 
clear


%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
MAP=int8(zeros(200,200));
MAP(1:200,1)=1;
MAP(200,1:200)=1;
MAP(1:200,200)=1;
MAP(1,1:200)=1;
MAP(130,1:95)=1;
MAP(1:60,95)=1;
MAP(90:130, 95)=1;
MAP(130:160,60)=1;
MAP(180:200,60)=1;
MAP(30:60,20:70)=1;
MAP(80:110,20:70)=1;
 for rd=14:.1:17
     th = linspace(0,2*pi,1000);
     r=150+rd*sin(th);
     c=150+rd*cos(th);
     r=round(r);
     c=round(c);
     r1=100+rd*sin(th);
     c1=150+rd*cos(th);
     r1=round(r1);
     c1=round(c1);
     MAP(sub2ind(size(MAP),r1,c1))=1;
     MAP(sub2ind(size(MAP),r,c))=1;
     
     r2=50+rd*sin(th);
     c2=150+rd*cos(th);
     r2=round(r2);
     c2=round(c2);
     MAP(sub2ind(size(MAP),r2,c2))=1;
  
 end

t = 0:0.1:2*pi+0.1;



%initial start positions
StartX=15;
StartY=165;
Connecting_Distance=1;

queue1 = [0 0 0 0]
queue1 = randperm(3,3)
queue1(4) = 0;
batteryx= [20:10:60]
batteryy= 220;
cnt = 0;
axis xy
plot(batteryx,batteryy, 'rx')
pause(2)
for j=1:4
    if queue1(j) == 1
        GoalRegister=int8(zeros(200,200));
        GoalRegister(170,160) = 1;
    
        else if queue1(j) == 2
            GoalRegister=int8(zeros(200,200));
            GoalRegister(140,170) = 1;
          
        else if queue1(j) ==3
            GoalRegister=int8(zeros(200,200));
            GoalRegister(75,60) = 1;
      
            else if queue1(j) == 0 %|| length(batteryx) < 4
            GoalRegister=int8(zeros(200,200));
            GoalRegister(165,15) = 1;
            end
            end
            end
    end
    
    

if j > 1
    StartX=OptimalPath(1,2);
    StartY=OptimalPath(1,1);
end
 

% Running PathFinder
OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance)
% End. 


SmoothingPath = [OptimalPath(:,2),OptimalPath(:,1)]

x = SmoothingPath;
y = x;

%plot(x(:,1), x(:,2), 'bo')


err = 9999;
threshold = 0.01;

alpha = 15;
beta = 0.499;

while(err>threshold)
    preY = y;
    for i=2:length(x(:,1))-1
       y(i,:) = y(i,:)+alpha*(x(i,:)-y(i,:));
       y(i,:) = y(i,:)+beta*(y(i-1,:)+y(i+1,:)-2*y(i,:)); 
      
    end    
    err = norm(preY-y);
end

hold on





if size(OptimalPath,2)>1
figure(1)
imagesc((MAP))
colormap(flipud(gray));
axis xy
hold on
battery1=plot(batteryx,batteryy,'x','color', 'red')

start1 = plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','blue')
legend([start1],{'Start'});
pause(2)
goal1 = plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
legend([start1, goal1],{'Start','Goal'});
pause(2)
hline3=plot(OptimalPath(:,2),OptimalPath(:,1),'b.')
pause(2)
delete(hline3)


if j==3
 for rd2=11:.1:14
     th = linspace(0,2*pi,1000);
     r3=165+rd2*sin(th);
     c3=80+rd2*cos(th);
     r3=round(r3);
     c3=round(c3);
     MAP(sub2ind(size(MAP),r3,c3))=1;
 end
end
if j==4
 OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance)

hline3=plot(OptimalPath(:,2),OptimalPath(:,1),'b.')
pause(2)
delete(hline3)

SmoothingPath = [OptimalPath(:,2),OptimalPath(:,1)]

x = SmoothingPath;
y = x;

%plot(x(:,1), x(:,2), 'bo')


err = 9999;
threshold = 0.01;

alpha = 15;
beta = 0.499;

while(err>threshold)
    preY = y;
    for i=2:length(x(:,1))-1
       y(i,:) = y(i,:)+alpha*(x(i,:)-y(i,:));
       y(i,:) = y(i,:)+beta*(y(i-1,:)+y(i+1,:)-2*y(i,:)); 
      
    end    
    err = norm(preY-y);
end

hold on

end

for i=length(y+4):-1:1
plot(y(i,1),y(i,2),'r.')
hold on


x1 = 5.*cos(t)+y(i,1);
y1 = 5.*sin(t)+y(i,2);
if i >6
hline = line([OptimalPath(i,2),OptimalPath(i-5,2)],[OptimalPath(i,1),OptimalPath(i-5,1)])
hline3 = line([OptimalPath(i,2),OptimalPath(i-5,2)],[OptimalPath(i,1),OptimalPath(i-5,1)])
end
hline2 =line(x1, y1);
drawnow
delete(hline)
delete(hline2)
delete(hline3)
save=i;
end
%hline = line([OptimalPath(i,2),OptimalPath(i-5,2)],[OptimalPath(i,1),OptimalPath(i-5,1)])
hline2 =line(x1, y1);
end
if queue1(j) == 1
        cnt = cnt+1;
        batteryx(6-cnt) =[]
        hold off
        plot(batteryx,batteryy, 'rx')
     else if queue1(j) == 2
            cnt = cnt+1;
            batteryx(6-cnt) =[]
            hold off
            plot(batteryx,batteryy, 'rx')
     else if queue1(j) ==3
            cnt = cnt+1;
            batteryx(6-cnt) =[]
            hold off
            plot(batteryx,batteryy, 'rx')
     else if queue1(j) ==0
            batteryx =[20:10:60]
            cnt = 0;
            plot(batteryx,batteryy, 'rx')
         end
         end
         end
end
    
end