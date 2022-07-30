// ACCEPTED

#include <bits/stdc++.h>

using namespace std;

typedef pair<double,double> Point;


double cross_product(Point a, Point b, Point p) {
    double xa = a.first;
    double ya = a.second;

    double xb = b.first;
    double yb = b.second;

    double xp = p.first;
    double yp = p.second;

    double z = (xb-xp) * (ya-yp) - (xa-xp) * (yb-yp);

    return z;
}

vector<pair<Point,Point>> convex_hull(vector<Point> points , long long n, int dir) {
    vector<pair<Point,Point>> valleys_points;
    vector<Point> upper;
    for (long long i = 1 ; i < n-1 ; ++i) {
        Point p =  points[i];
        while (upper.size() >= 2 && dir * cross_product( *(upper.end()-2),upper.back(),p) < 0)
            upper.pop_back();

        upper.push_back(p);

        if (i % 2 == 0) valleys_points.push_back({*(upper.end()-2),upper.back()});
    }
    return valleys_points;
}

double compute_intersection(pair<Point,Point> ab, long long y){
    Point a = ab.first;
    Point b = ab.second;

    double xa = a.first;
    double ya = a.second;

    double xb = b.first;
    double yb = b.second;

    double slope = 0;
    if (xa-xb != 0) slope = (ya-yb) / (xa-xb);
    double cte_b = ya - slope * xa;

    double x = y - cte_b;
    if (slope != 0) x = (y-cte_b) / slope;

    return x;
}

vector<pair<double, double>> get_segments(vector<pair<Point, Point>> valleys_points_left_side, vector<pair<Point, Point>> valleys_points_right_side, long long yf){
    vector<pair<double, double>> segments;
    long long nb_valleys = valleys_points_left_side.size();
    for (long long i = 0 ; i < nb_valleys ; ++i) {
        double x_left = compute_intersection(valleys_points_left_side[i],yf);
        double x_right = compute_intersection(valleys_points_right_side[nb_valleys-i-1], yf);
        segments.push_back({x_left,x_right});
    }
    return segments;
}

long long minimum_number_of_fairies(vector<pair<double, double>> segments) {
    long long n = segments.size();
    sort(segments.begin() , segments.end() , [](Point p1, Point p2) {return p1.second < p2.second;});
    long long ans = 1;
    double p = segments[0].second;
    for (long long i = 1 ; i < n ; ++i){
        if (p < segments[i].first ) {
            ans++;
            p = segments[i].second;
        }
    }

    return ans;
}

int main(){
    ios::sync_with_stdio(false);
    cin.tie(NULL);
    cout.tie(NULL);

    long long n,yf;
    cin >> n >> yf;

    vector<Point> points;
    for(long long i = 0; i < n ; ++i){
        double x,y;
        cin >> x >> y;
        points.push_back({x,y});
    }

    /*
    * Subtask #1: determine valleys points
    */
    vector<pair<Point,Point>> valleys_points_left_side = convex_hull(points,n,1);

    reverse(points.begin() , points.end());
    vector<pair<Point, Point>> valleys_points_right_side = convex_hull(points, n, -1);


    /*
    *Subtast #2: Get all the segments of all valleys
    */          
    vector<pair<double, double>> segments = get_segments(valleys_points_left_side, valleys_points_right_side,yf);

    
    /*
    * Minimum number of intersetions of the computed set of segments
    */

    long long ans = minimum_number_of_fairies(segments);

    cout << ans << '\n';
    return 0;
}
