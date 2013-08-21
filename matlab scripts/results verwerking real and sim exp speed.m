%loop over data
rawnames={'real1raw.txt';'real2raw.txt';'real3raw.txt';'real4raw.txt';'real5raw.txt';'real6raw.txt';'real7raw.txt';'real8raw.txt';'real9raw.txt'};
imagenames={'real1.png';'real2.png';'real3.png';'sim1.png';'sim2.png';'sim3.png';};
filenames={'real1.txt';'real4.txt';'real7.txt';'real2.txt';'real5.txt';'real8.txt';'real3.txt';'real6.txt';'real9.txt';'sim1.txt';'sim2.txt';'sim3.txt';'sim4.txt';'sim5.txt';'sim6.txt';'sim7.txt';'sim8.txt';'sim9.txt'};



color = [1 0 0; 0 1 0; 0 0 1];
for a=1:9
	data1=load(rawnames{a});
	data=data1(2:end,1:2);
	for i=1:length(data)
		%if current is -1 search for next -1 take data before and after and average over time spots
		if data(i,1) == -1
			j = i;
			if i == 1
				while j<length(data) && data(j,1)==-1
					j=j+1;
				end
				for k=i:1:j-1
					data(k,1)=data(j,1);
					data(k,2)=data(j,2);
				end
			else
				while j<length(data) && data(j,1)==-1
					j=j+1;
				end
				if j==length(data)
					for k=i:1:j
						data(k,1)=data(i-1,1);
						data(k,2)=data(i-1,2);
					end
				else
					diff=j-(i-1);
					xdiff=data(j,1)-data(i-1,1);
					ydiff=data(j,2)-data(i-1,2);
					for k=i:1:j-1
						data(k,1)=data(i-1,1)+((xdiff/diff)*(k-(i-1)));
						data(k,2)=data(i-1,2)+((ydiff/diff)*(k-(i-1)));
					end
				end
			end
			
			
			i=j;
		end
	end


	for i=1:length(data)
		data(i,1) = (((data(i,1)-0.0674)*1.44325)-0.61);
		data(i,2) = (((data(i,2)-0.1487)*0.75220)-0.26);
	end
	
	csvwrite(filenames{a},data);
end


%loop over data
for a=1:18
	data=load(filenames{a});
	results=[];
	resultssec=[];
	secdist=0;
	for i=2:length(data)

		%take this one - previous one to find distance covered
		results(i-1)=sqrt((data(i,1)-data(i-1,1))^2+(data(i,2)-data(i-1,2))^2);
		secdist=secdist+results(i-1);
		%total per sec
		rem=mod(i-1,10);
		if rem==0
			resultssec((i-1)/10)=secdist;
			secdist=0;
		end
	end
		rem=mod(a-1,3);
		if rem==0
		f=figure('Visible','off');
		end
		plot(resultssec(:), 'color', color(rem+1,:), 'LineWidth',2)
		axis([0 30 0 0.16])
		hold on
		if rem==2
		print(f,'-dpng',imagenames{a/3})
		end
end