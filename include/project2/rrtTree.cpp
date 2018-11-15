#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
    root->cnt=0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i < count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	    for(int j = 0; j < 10; j++) {
	        double alpha = path[i].alpha;
	        double d = path[i].d;
	        double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	        double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	        double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {

    //TODO
        ptrTable[count] = new node;
        ptrTable[count]->idx = count;
    ptrTable[count]->idx_parent = idx_near;
    ptrTable[count]->location= x_new;
    ptrTable[count]->rand = x_rand;
    ptrTable[count]->alpha = alpha;
        ptrTable[count]->d = d;
	ptrTable[count]->cnt = 0;
        count++;


}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
        for(int i=0; i<K; i++){	
//		if(!(i%100)) printf("i : %d \n ", i );
		if(pow(ptrTable[count-1]->location.x-x_goal.x,2)+pow(ptrTable[count-1]->location.y-x_goal.y,2)<pow(0.1,2)) break;
		int passed=0;
//		printf("i : %d\n",i);
                point x_rand = randomState(x_max, x_min, y_max, y_min);
//		printf("randomState : %d\n",i);
                int idx_near = nearestNeighbor(x_rand,MaxStep);
//		printf("nearestNeighbor : %d\n",i);
		double out[5];
//		printf("double out[5]\n");
//		printf("%d \n",idx_near);
		
		while(!passed) {
		if(idx_near!=-1) {
                if(randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep)){
//		printf("randompath : %d\n",i);
		point x_new;
		ptrTable[idx_near]->cnt=0;
		x_new.x = out[0];
		x_new.y = out[1];
		x_new.th = out[2];
                double alpha = out[3];
                double d = out[4];
                this->addVertex(x_new, x_rand, idx_near, alpha, d);
//		printf("addVertex : %d\n",i);
		passed = 1;
		}
		else
		{
		    ptrTable[idx_near]->cnt++;
		    if(idx_near==0) {
			idx_near=rand()%count;	
			break;
		    }
		    if(ptrTable[idx_near]->cnt>=4)
		    {
			idx_near=ptrTable[idx_near]->idx_parent;
//		        ptrTable[idx_near]->location.x = 10000000;
//			ptrTable[idx_near]->location.y = 10000000;
		    }
		}
		if(!idx_near)idx_near = ptrTable[idx_near]->idx_parent;
		//     printf("idx_near %d\n",idx_near);
		}
		else break;
	    }
        }
	return 0;

}
point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {

    point random;
    random.x=rand()%((int)x_max - (int)x_min)+x_min;
    if(random.x>x_max) random.x=x_max;

    random.y=rand()%((int)y_max - (int)y_min)+y_min;
    if(random.y>y_max) random.y=y_max;

    random.th=atan2(random.y,random.x);
    if(count%5==0) return x_goal;
    return random;
    //TOOD
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    double min_distance=100000000;
    double R=L/tan(max_alpha);
    int index_min=-1;
    for(int i=0;i<count;i++)
    {
        double x_c1 = ptrTable[i]->location.x+R*sin(ptrTable[i]->location.th);
	double y_c1 = ptrTable[i]->location.y-R*cos(ptrTable[i]->location.th);
	double x_c2 = ptrTable[i]->location.x-R*sin(ptrTable[i]->location.th);
	double y_c2 = ptrTable[i]->location.y+R*sin(ptrTable[i]->location.th);
//	if(fabs( ptrTable[i]->location.th - atan2(ptrTable[i]->location.y-x_rand.y,ptrTable[i]->location.x-x_rand.x)      )<=2*max_alpha   )        {
//	if( fabs( ptrTable[i]->location.th - atan2(ptrTable[i]->location.y-x_rand.y,ptrTable[i]->location.x-x_rand.x))<1.57 ) {
	if(pow(x_c1-x_rand.x,2)+pow(y_c1-x_rand.y,2) > pow(R,2) && pow(x_c2-x_rand.x,2) + pow(y_c2-x_rand.y,2) > pow(R,2)) {
            double tmp=pow(x_rand.x-ptrTable[i]->location.x,2)+pow(x_rand.y-ptrTable[i]->location.y,2);
            if(tmp<min_distance)
            {
                min_distance=tmp;
                index_min=i;
            }
        }
    }
    return index_min;
    //TODO
}


int rrtTree::nearestNeighbor(point x_rand) {
    //TODO	
	int neighbor=0;
	double shortest=pow(x_rand.x-ptrTable[0]->location.x,2)+pow(x_rand.y-ptrTable[0]->location.y,2);

	for(int i=0;i<count;i++)
	{	
		double tmp=pow(x_rand.x-ptrTable[i]->location.x,2)+pow(x_rand.y-ptrTable[i]->location.y,2);
		if(tmp<shortest)
		{
			shortest=tmp;
			neighbor=i;
		}
		

	}	
	return neighbor;
}


int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep) {
	
//	printf("randompath start! \n");
    //TODO
        int n=5;
        double *alpha = new double[n];
        double *d = new double[n];
        double *R = new double[n];
        double *x_c = new double[n];
        double *y_c = new double[n];
        double *x_ = new double[n];
        double *y_ = new double[n];
        double *th_ = new double[n];
	point *p = new point[n];
//	printf("after allocation \n");

        for(int i=0;i<n;i++){
//		printf("i : %d\n",i);
	        alpha[i] =( ((double)rand()/RAND_MAX)*2-1 )*max_alpha;
                d[i] = ( (fabs((double)rand()/RAND_MAX))*0.35+0.65 )*MaxStep;
//                if(i%10==0) d[i] = MaxStep;
 	//	if(count %5==0) d[i]=0;

                R[i] = L/tan(alpha[i]);
                x_c[i] = x_near.x - R[i]*sin(x_near.th);
                y_c[i] = x_near.y + R[i]*cos(x_near.th);
                x_[i] = x_c[i] + R[i]*sin(x_near.th + d[i]/R[i]);
                y_[i] = y_c[i] - R[i]*cos(x_near.th + d[i]/R[i]);
                th_[i] = x_near.th + d[i]/R[i];
		p[i].x = x_[i];
		p[i].y = y_[i];
		p[i].th = th_[i];
        }

	
        double d2 = (x_rand.x-x_[0])*(x_rand.x-x_[0]) + (x_rand.y-y_[0])*(x_rand.y-y_[0]);
        int optimal = n;
        for(int i=0;i<n;i++){
		if(!isCollision(x_near, p[i], d[i], R[i])){
                double tmp = (x_rand.x-x_[i])*(x_rand.x-x_[i]) + (x_rand.y-y_[i])*(x_rand.y-y_[i]);
                if(tmp < d2 ){
                        d2 = tmp;
                        optimal = i;
                }
		}
        }
//	printf("out allocation\n");
        out[0] = x_[optimal];
        out[1] = y_[optimal];
        out[2] = th_[optimal];
        out[3] = alpha[optimal];
        out[4] = d[optimal];
	double R_min=L/tan(max_alpha);
/*	if(x_goal.x==x_rand.x && x_goal.y==x_rand.y)
	{
            double x_c1 = x_near.x+R_min*sin(x_near.th);
    	    double y_c1 = x_near.y-R_min*cos(x_near.th);
	    double x_c2 = x_near.x-R_min*sin(x_near.th);
	    double y_c2 = x_near.y+R_min*sin(x_near.th);
        if(pow(x_goal.x-x_near.x,2)+pow(x_goal.y-x_near.y,2)<=pow(MaxStep,2) && (pow(x_c1-x_goal.x,2)+pow(y_c1-x_goal.y,2) > pow(R_min,2) && pow(x_c2-x_goal.x,2) + pow(y_c2-x_goal.y,2) > pow(R_min,2))) {
		out[0]=x_goal.x;
		out[1]=x_goal.y;
		out[4]=sqrt(pow(x_rand.x-x_near.x,2)+pow(x_rand.y-x_near.y,2));
		double R_goal=1/(1.57-atan((x_goal.y-x_near.y)/(x_goal.x-x_near.x)))*out[4];
		out[2]=x_near.th+out[4]/R_goal;
		out[3]=atan(R_goal/L);
	}
	}*/
       // if(isCollision(x_near, x_new, d[optimal], R[optimal])){
	//	printf("isCollision \n");
                delete [] alpha;
                delete [] d;
                delete [] R;
                delete [] x_c;
                delete [] y_c;
                delete [] x_;
                delete [] y_;
                delete [] th_;
		delete [] p;
	//	printf("after delete\n");
	if(optimal==n)        
        return 0;
	else	return 1;
        
	/*
        else{
		printf("iscollision\n");
                delete [] alpha;
                delete [] d;
                delete [] R;
                delete [] x_c;
                delete [] y_c;
                delete [] x_;
                delete [] y_;
                delete [] th_;
		printf("after delete\n");
                return 1;
        }*/

}


bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    //TODO
	double alpha = atan(L/R);
	double x_center = x1.x-R*sin(x1.th);
	double y_center = x1.y+R*cos(x1.th);
	double beta = d/R;
	bool result = true;
	int x_origin = int(map_origin_x);
	int y_origin = int(map_origin_y);
	for (int n=1;n<=10;n++)
	{
		double x_path=x_center+R*sin(x1.th+beta/10*n);
		double y_path=y_center-R*cos(x1.th+beta/10*n);
		
//	printf("iscollision x %f y %f\n",(x_path/0.05+x_origin),(y_path/0.05+y_origin));	
		for(int i=-4;i<4;i++)
		{
			for(int j=-4;j<4;j++)
			{	
				result = result && map.at<uchar>(i+x_path/0.05+x_origin, j+y_path/0.05+y_origin);
			}
		}
		
	}
	return !result;
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
//	printf("before nearestNeighbor\n");
	int near_goal = nearestNeighbor(x_goal);
//	printf("nearestNeighbor in backtracking\n");
	std::vector<traj> path;
//	printf("vector definition\n");
//	printf("near goal %d\n",near_goal);
	traj nearG;
	nearG.x =  ptrTable[near_goal]->location.x;
//	printf("ptrtable1\n");
	
        nearG.y =  ptrTable[near_goal]->location.y;
//	printf("ptrtable1\n");
	
        nearG.th =  ptrTable[near_goal]->location.th;
//	printf("ptrtable1\n");
	
        nearG.alpha =  ptrTable[near_goal]->alpha;
//	printf("ptrtable1\n");
	
        nearG.d =  ptrTable[near_goal]->d;
//	printf("ptrtable1\n");
	
	path.push_back(nearG);
//	printf("push_back\n");
	
	int now = near_goal;
	while(now!=0){
//		printf("while\n");
		traj trj;
		int prt = ptrTable[now]->idx_parent;
		trj.x = ptrTable[prt]->location.x;
		trj.y = ptrTable[prt]->location.y; 
		trj.th = ptrTable[prt]->location.th; 
		trj.alpha = ptrTable[prt]->alpha; 
		trj.d = ptrTable[prt]->d;
		path.push_back(trj);
		now = prt;
	}
	return path;
}
