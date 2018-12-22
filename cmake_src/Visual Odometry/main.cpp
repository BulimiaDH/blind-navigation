#include <Odometry.h>


int main()
{
	Odometry* odometry = new Odometry();
	odometry->runDissectingScale("output.txt");
	delete odometry;
}
 
