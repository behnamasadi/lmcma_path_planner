// Limited Memory Covariance Matrix Adaptation Evolution Strategy (LM-CMA-ES)
// submitted to GECCO-2014

typedef struct
{
    double value;
    int id;
} sortedvals;

typedef struct
/* random_t
 * sets up a pseudo random number generator instance
 */
{
    /* Variables for Uniform() */
    long int startseed;
    long int aktseed;
    long int aktrand;
    long int* rgrand;

    /* Variables for Gauss() */
    short flgstored;
    double hold;
} random_t;

/*
 * random_Start(), random_init(), random_exit(), random_Uniform(),
 * random_Gauss(), time_tic(), time_tictoc(), time_toc() are adopted
 * from Nikolaus Hansen's source code for CMA-ES
 */

void random_exit(random_t* t);
long random_Start(random_t* t, long unsigned inseed);
long random_init(random_t* t, long unsigned inseed);
double random_Uniform(random_t* t);
double random_Gauss(random_t* t);
int compare(const void* a, const void* b);
void myqsort(int sz, double* arfitness, int* arindex, sortedvals* arr);


class CMABase
{
protected:
    int N;
    int lambda;
    int mu;
    double mueff;
    double* initialParams;
    double* loBounds;
    double* hiBounds;
    double* arx;
    double* arfitness;
    double* xmean;
    double* xold;
    double* weights;
    double* covariance;
    int inseed;
    bool verbose;
    double* L;
    int itr;
    int sampleIdx;
    random_t ttime;
public:
    int counteval;
    double BestF;

    CMABase(double* initialParams, int lambda, double* loBounds,
            double* hiBounds, double* covariance, int inseed,
            bool verbose=false);
    virtual ~CMABase();
    virtual void init(int N);
    void getNextParameterVector(double* params, int N);
    void setEvaluationFeedback(double* feedbacks, int numFeedbacks);
    virtual bool isBehaviorLearningDone();

protected:
    virtual void update() = 0;
    virtual void sample() = 0;
    void sampleStandardNormal(double* z);
    void applyBoundaries();
};


/**
 * @class LMCMA
 *
 * Limited-memory CMA-ES.
 */
class LMCMA : public CMABase
{
    double sigma;
    int nvectors;
    int maxsteps;
    double c1;
    double cc;
    double cs;
    double val_target; //!< Target population success ratio
    double* v_arr;
    double* pc_arr;
    double* pc;
    double* z;
    double* Az;
    double* Av;
    int* iterator;
    double* prev_arfitness;
    int* arindex;
    double* mixed;
    int* ranks;
    int* ranks_tmp;
    double* Nj_arr;
    double* Lj_arr;
    sortedvals* arr_tmp;
    int* t;
    int* vec;

    double K, M;
    double s;
    int iterator_sz;
public:
    /**
     * LMCMA constructor.
     * @param initialParams initial parameters
     * @param lambda population size, defaults to 4 + int(3 * log(n)) if < 1
     * @param loBounds lower boundaries per parameter, optional
     * @param hiBounds upper boundaries per parameter, optional
     * @param sigma initial exploration size, defaults to 1
     * @param covariance a fixed initial covariance matrix
     * @param inseed seed for random number generator
     * @param verbose print debug infos
     */
    LMCMA(double* initialParams, int lambda=0, double* loBounds=0,
          double* hiBounds=0, double sigma=1.0, double* covariance=0,
          int inseed=0, bool verbose=false);
    ~LMCMA();
    void init(int N);
    void sample();
    void update();
    bool isBehaviorLearningDone();

private:
    void computeAz(double* Az, double* z);
    void invAz(int N, double* Av, int iterator_sz, int* iterator,
               double* v_arr, double* Lj_arr, double K);
};


/**
 * @class SepCMA
 *
 * Separable CMA-ES.
 */
class SepCMA : public CMABase
{
    double cc;
    double sigma;
    double* arz;
    double* diagD;
    double* diagC;
    double* pc;
    double* ps;
    int* arindex;
    sortedvals* arr_tmp;
    double c1;
    double cmu;
    double ccov1_sep;
    double ccovmu_sep;
    double chiN;
    double cs;
    double damps;
public:
    /**
     * SepCMA constructor.
     * @param initialParams initial parameters
     * @param lambda population size, defaults to 4 + int(3 * log(n)) if < 1
     * @param loBounds lower boundaries per parameter, optional
     * @param hiBounds upper boundaries per parameter, optional
     * @param sigma initial exploration size, defaults to 1
     * @param covariance a fixed initial covariance matrix
     * @param inseed seed for random number generator
     * @param verbose print debug infos
     */
    SepCMA(double* initialParams, int lambda=0, double* loBounds=0,
           double* hiBounds=0, double sigma=1.0, double* covariance=0,
           int inseed=0, bool verbose=false);
    ~SepCMA();
    void init(int N);
    void sample();
    void update();
};


/**
 * @class CMAChol
 *
 * Cholesky-CMA-ES.
 *
 * Note: might sometimes produce unstable results with too tight boundaries.
 */
class CMAChol : public CMABase
{
    double sigma;
    double* A;
    double* Ainv;
    double* arz;
    double* pc;
    double* ps;
    double* zmean;
    double* tmp_vec;
    double* tmp_vec2;
    int* arindex;
    sortedvals* arr_tmp;
    double cc;
    double c1;
    double chiN;
    double cs;
    double damps;
public:
    /**
     * CMAChol constructor.
     * @param initialParams initial parameters
     * @param lambda population size, defaults to 4 + int(3 * log(n)) if < 1
     * @param loBounds lower boundaries per parameter, optional
     * @param hiBounds upper boundaries per parameter, optional
     * @param sigma initial exploration size, defaults to 1
     * @param covariance a fixed initial covariance matrix
     * @param inseed seed for random number generator
     * @param verbose print debug infos
     */
    CMAChol(double* initialParams, int lambda=0, double* loBounds=0,
            double* hiBounds=0, double sigma=1.0, double* covariance=0,
            int inseed=0, bool verbose=false);
    ~CMAChol();
    void init(int N);
    void sample();
    void update();
private:
    /** (1+1)-Cholesky-CMA */
    void CholeskyUpdate(double* pc, double* Ainv, double* A);
};

/**
 * Create a covariance that minimizes the accelerations of a trajectory.
 * @param num_dims number of trajectory dimensions, e.g. 6 for Cartesian poses
 * @param num_waypoints number of waypoints of the trajectory
 * @param covariance allocated array of size (num_dims * num_waypoints)^2,
 *                   will be filled with values
 */
void covariance(int num_dims, int num_waypoints, double* covariance);

void differentiationMatrix(int num_time_steps, int order, double dt,
                           double* diff_matrix, int rowLen=-1);
void invert(double* A, double* Ainv, int N);
void cholesky(double* C, double* L, int N);
void applyCovL(double* L, double* z, int N);

