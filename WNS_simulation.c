#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

typedef struct 
{
  int x; /* x-coordinate of sensor node location in target field */
  int y; /* y-coordinate of sensor node location in target field */
  int *keyring; /* key ring */
  int phynbrsize; /* number of physical neighbors of a sensor node */
  int keynbrsize; /* number of key neighbors of a sensor node */
  int *phynbr; /* List of physical neighbors of a sensor node */
  int *keynbr; /* List of key neighbors of a sensor node */
} sensor;
int n,total_path_key_one_hop_count=0,total_path_key_two_hop_count =0;
sensor sensor_node[1000000];
float average_neighbour_size=0.0,total_key=0.0, total_physical=0.0;
void finding_network_connectivity (int keyring_size, int keypool_size)
{
  float num_one_hop, num_two_hop=0.0,temp, p_0=0.0, p_1, p_2,pro=1.0, num, den, peg;
  int count=0,i;
  printf("Network Connectivities : \n");
  for(i=0; i<n; ++i)
  {
    total_key +=sensor_node[i].keynbrsize;
    total_physical +=sensor_node[i].phynbrsize;
  }
printf("\nSimulated Average Network Connectivity\n%f\nTheoritical Network Connectivity\n", total_key/total_physical);
for (i=0; i<keyring_size ;++i)
      {
        temp=keypool_size - i;
        num = temp - keyring_size;
        pro *= (num/temp);
      }
  p_0 = 1-pro;
  p_1 = pow((1-pow(p_0,2)), average_neighbour_size);
  p_1 = (1 - p_0) * p_1;
  p_1 = 1 - p_1;
  p_2 = pow((1-(p_0 * p_1)), average_neighbour_size);
  p_2 *=(1-p_1);
  printf("%f\n", p_0);
  printf("Simulated Average Network Connectivity for one hop\n%f\n", (total_path_key_one_hop_count + total_key)/total_physical);
  printf("Theoritical Average Network Connectivity for one hop\n%f\n", p_1 );
  printf("Simulated Average Network Connectivity for two hop\n%f\n", (total_path_key_one_hop_count + total_path_key_two_hop_count + total_key)/total_physical);
  printf("Theoritical Average Network Connectivity for two hop\n%f\n",1 - p_2);
}
double distance_cal(int x1, int y1, int x2, int y2)
{  

  double diffx,diffy,diffx_sqr,diffy_sqr;
  diffx = x1 - x2;
  diffx_sqr = pow(diffx, 2);
  diffy = y1 - y2;
  diffy_sqr = pow(diffy, 2); 
  return sqrt (diffx_sqr + diffy_sqr);
}
simulating_path_key_establishment(int keyring_size,int keypool_size)
{
  int arr[10000], temp_physical[1000], one_hop[1000], one_hop_temp[1000],i, j , k, x, y, index, z, w;
  for(i=0; i<n; ++i)
  {
    int t=0,flag=0,one_hop_index = 0,one_t=0;;
    for(x=0; x<sensor_node[i].phynbrsize; ++x)
   {
     arr[x] = *(sensor_node[i].phynbr+x);
   }
   for(j=0; j<sensor_node[i].keynbrsize; ++j)
    {
      int key_nbr_j=*(sensor_node[i].keynbr+j);
      for(k=0; k<sensor_node[i].phynbrsize; ++k)
      {
        if( key_nbr_j==arr[k])
        {
          arr[k]=-1;
        }

      }
    }
    index=0;
    for(z=0; z<sensor_node[i].phynbrsize; ++z)
    {
      if(arr[z]==-1)
        continue;
      temp_physical[index] = arr[z];
      ++index;      
    }
    //one-hop-calculation
    for(t=0; t<index; ++t)
    {
      flag=0;
      int temp_keynbrsize=sensor_node[i].keynbrsize;
      for(y=0; y<temp_keynbrsize; ++y)
      {
      int temp_yth_keynbr=*(sensor_node[i].keynbr+y);
        for(z=0; z<sensor_node[temp_yth_keynbr].keynbrsize;++z)
        {
          if((*(sensor_node[temp_yth_keynbr].keynbr+z))==temp_physical[t])
          {
              flag=1;
              one_hop[one_hop_index++]= temp_physical[t];
              ++total_path_key_one_hop_count;
              break;
          }

        }
        if(flag==1)
          break;

      }

    }
    //two-hop temporary array creation
      for(x=0; x<one_hop_index; ++x)
      {
         for(y=0; y<index; ++y)
         {
            if(one_hop[x]!=temp_physical[y])
              continue;
            temp_physical[y]=-1;

         }
      }
      one_t=0;
      for (x = 0; x < index; ++x)
      {
        if(temp_physical[x]>-1)
        {
          one_hop_temp[one_t++]=temp_physical[x];
        }

        /* code */
      }
      /* Two hop counts calculation */
      for(x=0; x<one_t; ++x)
      {
        flag=0;
        int temp_key_nbr_size=sensor_node[i].keynbrsize;
        for(y=0; y<temp_key_nbr_size; ++y)
        {
          int for_y_keynbr=*(sensor_node[i].keynbr+y);
          for(z=0; z<sensor_node[for_y_keynbr].keynbrsize; ++z)
          {
            int for_z_keynbr=*(sensor_node[for_y_keynbr].keynbr+z);
            for(w=0; w<sensor_node[for_z_keynbr].keynbrsize; ++w)
            {
              if(*(sensor_node[for_z_keynbr].keynbr+w)== one_hop_temp[x])
              {
                flag=1;
                ++total_path_key_two_hop_count;
                break;
              }
            }
            if(flag!=0)
              break;

          }
          if(flag!=0)
            break;
        }

      }

    }

}
finding_key_neighbours(int keyring_size)
{
  printf("EG scheme\nDistributing keys...\n");
	int key_neighbours_array[100000],i,flag=0;
	for(i=0; i<n; ++i)
	{ 
		sensor_node[i].keynbrsize=0;
		int x;
		for(x=0; x<sensor_node[i].phynbrsize; ++x)
		{
			int physical_neighbour_id,y,k;
			physical_neighbour_id = *(sensor_node[i].phynbr+x);
			flag=0;
			for (y=0 ; y<keyring_size; ++y)
			{
				for (k= 0; k < keyring_size; ++k)
				{
					if(*(sensor_node[physical_neighbour_id].keyring+k)==*(sensor_node[i].keyring+y))
					{
						key_neighbours_array[sensor_node[i].keynbrsize] = physical_neighbour_id;
						++sensor_node[i].keynbrsize;
						flag=1;
						break;
					}
				}
				if(flag!=0) break;
			}

        }
        int z;
        sensor_node[i].keynbr   =(int*)malloc(sensor_node[i].keynbrsize*sizeof(int));
        for(z=0; z<sensor_node[i].keynbrsize; ++z)
        {
         *(sensor_node[i].keynbr+z) = key_neighbours_array[z];
        }
	}
}

/*generating_random_XY( int keyring_size,FILE *fp,int keypool_size)
{
	char str[100];
	memset(str, '\0', sizeof(str));
	sprintf(str, "%d", n);
  	fprintf(fp,"%s\n", str);
  	time_t t;
	srand((unsigned) time(&t));
  	int i=0, j=0,x,y,flag=0, key1,n1, n2;
	for(i=0; i<n; ++i)
	{
		
		//genrate key from pool 
		for (x = 0; sensor_node[i].phynbrsize;++x)
		{
			int fkey_ring=0;
			key1 = rand() % keypool_size + 1;
			for(y=0; y<x; ++y)
            {
            	if (*(sensor_node[i].keyring + y) == key1)
            	{
            		fkey_ring=1;
            		--x;
            		break;             
                }
           }
           if(fkey_ring==0)
           {
           	*(sensor_node[i].keyring + x) = key1;
           }
		}


		flag=0;
		n1 = rand() % 500 + 1;
        n2 = rand() % 500 + 1;
		sensor_node[i].keyring =(int*)malloc(keyring_size*sizeof(int));
		for(j=0; j<i; ++j)
        {
        	if(sensor_node[j].x == n1 && sensor_node[j].y == n2)
        	{
        		--i;
        		flag=1;
        		break;
        	}
        }
        if(flag==0)
        {			
 			sensor_node[i].x = n1;
        	sprintf(str, "%d", n1);
        	fprintf(fp,"%s ", str);
        	sensor_node[i].y = n2;
        	sprintf(str, "%d", n2);
          	fprintf(fp,"%s\n", str);
        }



	}
fclose(fp);
return;	
}*/
void generating_random_numbers(int keyring_size,FILE *fp2, int keypool_size)
{
  
  char str[15];
  int i, j, x, y, index, n1, n2,flag=0, key1,fkey_ring=0;
  time_t t;
  srand((unsigned) time(&t));
  sprintf(str, "%d", n);
  fprintf(fp2,"%s\n", str);
  //allocation of x-y values and keyring
  for(i=0; i<n; ++i)
            {
            n1 = rand() % 500 + 1;
            n2 = rand() % 500 + 1; 	
            flag=0;
            for(j=0; j<i; ++j)
                    {
                      if(sensor_node[j].x == n1 && sensor_node[j].y == n2)
                              {
                              	--i;
                                flag=1;
                                break;
                              }
                    }
            if(flag==0)
                  {
                      sensor_node[i].x = n1;
                      sprintf(str, "%d", n1);
                      fprintf(fp2,"%s ", str);
                      sensor_node[i].y = n2;
	                  sprintf(str, "%d", n2);
                      fprintf(fp2,"%s\n", str);
                  }           
            sensor_node[i].keyring =(int*)malloc(keyring_size*sizeof(int));
            for(x=0; x<keyring_size; ++x)
                {
                  key1 = rand() % keypool_size + 1;
                  fkey_ring=0;
                  for(y=0; y<x; ++y)
                      {
                      if (*(sensor_node[i].keyring + y) == key1)
                            {
                              fkey_ring=1;
                              --x;
                              break;
                            }    
                      }
                  if(fkey_ring==0)
                      *(sensor_node[i].keyring + x) = key1;
                }
            }
  fclose(fp2);
}
void finding_physical_and_key_neighbours(int keyring_size)
{

	double distance ,total_distance=0.0,physical_neighbour_size=0.0;
	int i=0, x=0;
	for(i=0; i<n; ++i)
        {
        	int phy_neighbours_array[100000]; 
        	sensor_node[i].phynbrsize=0;
        	int j;
        	for (j = 0; j < n; ++j)
        	{
        		if (i==j)
        		{
        			continue;
        		}
        		int x1,x2,y1,y2;
        		x1=sensor_node[i].x;
        		y1=sensor_node[i].y;
        		x2=sensor_node[j].x;
        		y2=sensor_node[j].y;
        		distance =distance_cal(x1, y1, x2,y2);

        		if(25.00>=distance)
        		{
        			
        			phy_neighbours_array[sensor_node[i].phynbrsize] = j;
        			++sensor_node[i].phynbrsize;
        		}
            if(j>i)
            total_distance = total_distance + distance;
        	}
        physical_neighbour_size=physical_neighbour_size+sensor_node[i].phynbrsize;
        sensor_node[i].phynbr  =(int*)malloc(sensor_node[i].phynbrsize*sizeof(int));
        int k;
        int phy_nbr_size=sensor_node[i].phynbrsize;
        for(k=0; k<phy_nbr_size; ++k)
         {
          *(sensor_node[i].phynbr+k) = phy_neighbours_array[k];
         }

        }
average_neighbour_size = physical_neighbour_size/n;        
printf("Scaling communication range...\nAverage distance = %f\nCommunication range of sensor nodes = 25.00\nComputing physical neighbors...\n",total_distance/((n-1)*(n/2) ));   
printf("Average neighborhood size = %f\n", average_neighbour_size);
finding_key_neighbours(keyring_size);        
}
void printing_sensor_nodes(int keyring_size)
{
	printf("Printing internal Structure\n");
	int i;
	for (i = 0; i < n; ++i)
	{
		printf("Node %d:-\n%d %d\n",i,sensor_node[i].x, sensor_node[i].y );
		printf("Genrated Key ring:-\n");
		int x;
		for(x=0; x<keyring_size; ++x)
    	{
           printf("%d", *(sensor_node[i].keyring+x));
           printf(" ");
        }
        printf("\nPhysical neighbors size %d\n physical neighbors are:-\n", sensor_node[i].phynbrsize);
        int phy_neighbours_size=sensor_node[i].phynbrsize;
        for(x=0; x<phy_neighbours_size; ++x)
    	{
           printf("%d", *(sensor_node[i].phynbr+x));
           printf(" ");
        }
        int key_neighbours_size=sensor_node[i].keynbrsize;
       printf("\nKey neighbors size %d\nkey neighbors are:-\n",key_neighbours_size);
       for(x=0; x<key_neighbours_size; ++x)
	   {
	                  printf("%d", *(sensor_node[i].keynbr+x) );
	     			  printf(" ");
	   }
      printf("\n");
     printf("One key count %d\nTwo key count %d\n",total_path_key_one_hop_count,total_path_key_two_hop_count);
         
	}
}

int main( int argc, char *argv[] )  
{
	if(argc==4)
	{
	printf("Enter the no of sensor nodes\n");
	int argument_2,argument_3;
	char argument_1[100],string[10000]="plot[0:500][0:500] \"";
	memset(argument_1, '\0', sizeof(argument_1));
	strcpy(argument_1,argv[1]);
	strcat(string, argument_1);
  strcat(string, "\" every ::1 using 1:2");
	scanf("%d",&n);
	FILE *fp1;
    fp1=fopen("kalpish", "w");
    fprintf(fp1,"%s", string);
    fclose(fp1);
    //fprintf(fp1,"%s", string);
	FILE *fp;
    fp=fopen(argument_1, "w");
    if(fp==NULL) 
      {
        perror("Error opening file.");
      }
    argument_2=atoi(argv[2]);
    argument_3=atoi(argv[3]);  
    generating_random_numbers(argument_2,fp,argument_3);
    finding_physical_and_key_neighbours(argument_2);
    //printing_sensor_nodes(argument_2);
    simulating_path_key_establishment(argument_2,argument_3);
    finding_network_connectivity(argument_2,argument_3);
    }

    else
    {
    	printf("invalid number of arguments\n");
    	exit(0);
    }
	system("gnuplot -p 'kalpish'"); 
}