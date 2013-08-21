%loop over data
rawnames={'real1raw.txt';'real2raw.txt';'real3raw.txt';'real4raw.txt';'real5raw.txt';'real6raw.txt';'real7raw.txt'};
imagenames={'real1.png';'real2.png';'real3.png';'real4.png';'real5.png';'real6.png';'real7.png';'sim1.png';'sim2.png';'sim3.png';'sim4.png';'sim5.png';'sim6.png';'sim7.png'};
filenames={'real1.txt';'real2.txt';'real3.txt';'real4.txt';'real5.txt';'real6.txt';'real7.txt';'sim1.txt';'sim2.txt';'sim3.txt';'sim4.txt';'sim5.txt';'sim6.txt';'sim7.txt'};

for a=1:7
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
for a=1:14
	data=load(filenames{a});
	results=[];
	resultssec=[];
	secdist=0;
	for i=2:length(data)

		%take this one - previous one to find distance covered
		results(i-1)=sqrt((data(i,1)-data(i-1,1))^2+(data(i,2)-data(i-1,2))^2);
		secdist=secdist+results(i-1);
		%total per sec
		rem=mod(i-1,100);
		if rem==0
			resultssec((i-1)/100)=secdist;
			secdist=0;
		end
	end

	average=mean(resultssec)

	%plot data
	f=figure('Visible','off');
	plot(resultssec(:))
	axis([0 180 0 0.7])
	print(f,'-dpng',imagenames{a})
end