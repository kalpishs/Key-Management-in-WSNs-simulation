**************************************************WNS simulation**************************************************************
						NAME - Kalpish Singhal	***************************************************************************************************************************

#Task-1

->No. of sensor nodes(n) are taken as an input from the user
->n unique sample points are generated in the range of(500 X 500).
-> ploting is done for the points using Gnuplot

#Task-2

-> Simulated Average Network Connectivity : Sigma(Key neighbors for all nodes) / Sigma(Physical Neighbours for all nodes). This
gives the network connectivity for the direct key establishment phase.

#Task -3

-> Simulated Average Network Connectivity for one hop : Sigma(Key neighbors for all nodes + one_hop_key_count) / 
Sigma(Physical Neighbours for all nodes). This gives the network connectivity for the path key establishment phase using one hop.
-> Simulated Average Network Connectivity for two hop : Sigma(Key neighbors for all nodes + one_hop_key_count + two_hop_key_count)
/ Sigma(Physical Neighbours for all nodes). This gives the network connectivity for the path key establishment phase using two hop.
==============================================================================================================================
 						Running the code
==============================================================================================================================

->compile using -lm to use math library.
	i.e, 
		$ gcc WNS_simulation.c -lm
->Runing need to pass 3 command line arguments <filename> <keyring_size> <keypool_Size> 
	i.e, 
		$./a.out sensor3.data 100 40000
==============================================================================================================================
 						    Output
==============================================================================================================================

Enter the no of sensor nodes
10000
Scaling communication range...
Average distance = 261.167575
Communication range of sensor nodes = 25.00
Computing physical neighbors...
Average neighborhood size = 75.194801
EG scheme
Distributing keys...
Network Connectivities : 

Simulated Average Network Connectivity
0.221582
Theoritical Network Connectivity
0.221685
Simulated Average Network Connectivity for one hop
0.903190
Theoritical Average Network Connectivity for one hop
0.982401
Simulated Average Network Connectivity for two hop
0.999830
Theoritical Average Network Connectivity for two hop
1.000000

