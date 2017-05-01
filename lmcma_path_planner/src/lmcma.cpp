#include "lmcma.hpp"
#include "math.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <eigen3/Eigen/Dense>


void random_exit(random_t* t)
{
    free(t->rgrand);
}

long random_Start(random_t* t, long unsigned inseed)
{
    long tmp;
    int i;
    t->flgstored = 0;
    t->startseed = inseed;
    if(inseed < 1)
        inseed = 1;
    t->aktseed = inseed;
    for(i = 39; i >= 0; --i)
    {
        tmp = t->aktseed / 127773;
        t->aktseed = 16807 * (t->aktseed - tmp * 127773) - 2836 * tmp;
        if(t->aktseed < 0) t->aktseed += 2147483647;
        if(i < 32)
            t->rgrand[i] = t->aktseed;
    }
    t->aktrand = t->rgrand[0];
    return inseed;
}

long random_init(random_t* t, long unsigned inseed)
{
    clock_t cloc = clock();
    t->flgstored = 0;
    t->rgrand = new long[32];
    if(inseed < 1)
    {
        while((long)(cloc - clock()) == 0)
            ;
        inseed = (long)abs(100.0 * time(NULL) + clock());
    }
    return random_Start(t, inseed);
}

double random_Uniform(random_t* t)
{
    long tmp;
    tmp = t->aktseed / 127773;
    t->aktseed = 16807 * (t->aktseed - tmp * 127773)
                 - 2836 * tmp;
    if(t->aktseed < 0)
        t->aktseed += 2147483647;
    tmp = t->aktrand / 67108865;
    t->aktrand = t->rgrand[tmp];
    t->rgrand[tmp] = t->aktseed;
    return (double)(t->aktrand) / (2.147483647e9);
}

double random_Gauss(random_t* t)
{
    double x1, x2, rquad, fac;
    if(t->flgstored)
    {
        t->flgstored = 0;
        return t->hold;
    }
    do
    {
        x1 = 2.0 * random_Uniform(t) - 1.0;
        x2 = 2.0 * random_Uniform(t) - 1.0;
        rquad = x1 * x1 + x2 * x2;
    }
    while(rquad >= 1 || rquad <= 0);
    fac = sqrt(-2.0 * log(rquad) / rquad);
    t->flgstored = 1;
    t->hold = fac * x1;
    return fac * x2;
}

int compare(const void* a, const void* b)
{
    if(((sortedvals*)a)->value < ((sortedvals*)b)->value) return -1;
    if(((sortedvals*)a)->value == ((sortedvals*)b)->value) return 0;
    if(((sortedvals*)a)->value > ((sortedvals*)b)->value) return 1;
}

void myqsort(int sz, double* arfitness, int* arindex, sortedvals* arr)
{
    for(int i = 0; i < sz; i++)
    {
        arr[i].value = arfitness[i];
        arr[i].id = i;
    }
    qsort(arr, sz, sizeof(sortedvals), compare);
    for(int i = 0; i < sz; i++)
    {
        arfitness[i] = arr[i].value;
        arindex[i] = arr[i].id;
    }
}


CMABase::CMABase(double* initialParams, int lambda, double* loBounds,
                 double* hiBounds, double* covariance, int inseed, bool verbose)
    : lambda(lambda), initialParams(initialParams), loBounds(loBounds),
      hiBounds(hiBounds), covariance(covariance), inseed(inseed),
      verbose(verbose), L(0), itr(0), sampleIdx(0), counteval(0),
      BestF(std::numeric_limits<double>::max())
{
    random_init(&ttime, inseed);
}

CMABase::~CMABase()
{
    random_exit(&ttime);

    delete[] arx;
    delete[] arfitness;
    delete[] xmean;
    delete[] xold;
    delete[] weights;
    if(L)
        delete[] L;
}

void CMABase::init(int N)
{
    this->N = N;

    if(lambda < 1)
        lambda = 4 + int(3 * log(N));
    mu = lambda / 2;

    arx = new double[N*lambda];
    arfitness = new double[lambda];
    xmean = new double[N];
    xold = new double[N];
    weights = new double[mu];

    double sum_weights = 0;
    for(int i = 0; i < mu; i++)
    {
        weights[i] = log(double(mu) + 0.5) - log(double(1 + i));
        sum_weights += weights[i];
    }
    mueff = 0.0;
    for(int i = 0; i < mu; i++)
    {
        weights[i] /= sum_weights;
        mueff += weights[i] * weights[i];
    }
    mueff = 1.0 / mueff;

    if(initialParams)
        for(int i = 0; i < N; i++)
            xmean[i] = initialParams[i];
    else
        for(int i = 0; i < N; i++)
            xmean[i] = random_Uniform(&ttime);

    if(covariance)
    {
        L = new double[N * N];
        cholesky(covariance, L, N);
    }
}

void CMABase::getNextParameterVector(double* params, int N)
{
    double temp;
    for(int d = 0; d < N; d++)
    {
        params[d] = arx[sampleIdx * N + d];
        temp=arx[sampleIdx * N + d];
//        std::cout<<"arx[sampleIdx * N + d]: " <<arx[sampleIdx * N + d] <<std::endl;
    }

}

void CMABase::setEvaluationFeedback(double* feedbacks, int numFeedbacks)
{
    double feedback = 0.0;
    for(int i = 0; i < numFeedbacks; i++)
        feedback += feedbacks[i];

    arfitness[sampleIdx] = feedback;
    counteval++;
    if(feedback < BestF || counteval == 1)
    {
        BestF = feedback;
        if(verbose)
            std::cout << "Functions evaluation #" << counteval
                      << ", value: " << feedback << std::endl;
    }
    if(++sampleIdx % lambda == 0)
    {
        update();
        sample();
        sampleIdx = 0;
    }
}

bool CMABase::isBehaviorLearningDone()
{
    return false;
}

void CMABase::sampleStandardNormal(double* z)
{
    for(int n = 0; n < N; n++)
        z[n] = random_Gauss(&ttime);
    if(L)
        applyCovL(L, z, N);
}

void CMABase::applyBoundaries()
{
    if(loBounds)
        for(int i = 0; i < lambda; i++)
            for(int n = 0; n < N; n++)
                arx[i * N + n] = std::max(arx[i * N + n], loBounds[n]);
    if(hiBounds)
        for(int i = 0; i < lambda; i++)
            for(int n = 0; n < N; n++)
                arx[i * N + n] = std::min(arx[i * N + n], hiBounds[n]);
}


LMCMA::LMCMA(double* initialParams, int lambda, double* loBounds,
      double* hiBounds, double sigma, double* covariance, int inseed,
      bool verbose)
    : CMABase(initialParams, lambda, loBounds, hiBounds, covariance, inseed,
              verbose),
      sigma(sigma), cs(0.3), val_target(0.25)
{}

LMCMA::~LMCMA()
{
    delete[] arr_tmp;
    delete[] pc;
    delete[] z;
    delete[] Az;
    delete[] iterator;
    delete[] v_arr;
    delete[] pc_arr;
    delete[] prev_arfitness;
    delete[] arindex;
    delete[] Av;
    delete[] t;
    delete[] vec;
    delete[] mixed;
    delete[] ranks;
    delete[] ranks_tmp;
    delete[] Nj_arr;
    delete[] Lj_arr;
}

void LMCMA::init(int N)
{
    CMABase::init(N);

    nvectors = lambda;
    maxsteps = nvectors;
    c1 = 1.0 / (10 * log(N + 1));
    cc = 1.0 / nvectors;

    K = 1 / sqrt(1 - c1);
    M = sqrt(1 - c1);

    v_arr = new double[N*(nvectors)];
    pc_arr = new double[N*(nvectors)];
    pc = new double[N];
    z = new double[N];
    Az = new double[N];
    Av = new double[N];
    iterator = new int[nvectors];
    prev_arfitness = new double[lambda];
    arindex = new int[lambda];
    mixed = new double[2*lambda];
    ranks = new int[2*lambda];
    ranks_tmp = new int[2*lambda];
    Nj_arr = new double[nvectors];
    Lj_arr = new double[nvectors];
    arr_tmp = new sortedvals[2*lambda];
    t = new int[nvectors];
    vec = new int[nvectors];

    for(int i = 0; i < N; i++)
        pc[i] = 0;

    s = 0.0;
    iterator_sz = 0;

    sample();
}

void LMCMA::sample()
{
    for(int i = 0; i < lambda; i++) // O(m*n)
    {
        sampleStandardNormal(z);    // O(n)
        computeAz(Az, z);           // O(m*n)
        for(int k = 0; k < N; k++)  // O(n)
            arx[i * N + k] = xmean[k] + sigma * Az[k];
    }
    applyBoundaries();
}

void LMCMA::update()
{
    myqsort(lambda, arfitness, arindex, arr_tmp);
    for(int i = 0; i < N; i++)
    {
        xold[i] = xmean[i];
        xmean[i] = 0;
    }
    for(int i = 0; i < mu; i++)
    {
        double* cur_x = &arx[arindex[i] * N];
        for(int j = 0; j < N; j++)
            xmean[j] += weights[i] * cur_x[j];
    }
    for(int i = 0; i < N; i++)
        pc[i] = (1 - cc) * pc[i] + sqrt(cc * (2 - cc) * mueff) *
            (xmean[i] - xold[i]) / sigma;

    // Update set
    int imin = 1;
    if(itr < nvectors)
        t[itr] = itr;
    else
    {
        int dmin = vec[t[1]] - vec[t[0]];
        for(int j = 1; j < nvectors - 1; j++)
        {
            int dcur = vec[t[j + 1]] - vec[t[j]];
            if(dcur < dmin)
            {
                dmin = dcur;
                imin = j + 1;
            }
        }
        if(dmin >= maxsteps)
            imin = 0;
        if(imin != nvectors - 1)
        {
            int sav = t[imin];
            for(int j = imin; j < nvectors - 1; j++)
                t[j] = t[j + 1];
            t[nvectors - 1] = sav;
        }
    }
    iterator_sz = itr + 1;
    if(iterator_sz > nvectors)
        iterator_sz = nvectors;
    for(int i = 0; i < iterator_sz; i++)
        iterator[i] = t[i];
    int newidx = t[iterator_sz - 1];

    vec[newidx] = itr;
    for(int i = 0; i < N; i++)
        pc_arr[newidx * N + i] = pc[i];
    /*
     * This procedure recomputes v vectors correctly, in the original
     * LM-CMA-ES they were outdated/corrupted. The procedure allows to
     * improve the performance on some problems (up to 20% on Ellipsoid
     * with D=128..512) and sometimes better on other problems
     */
    if(imin == 1)
        imin = 0;
    for(int i = imin; i < iterator_sz; i++)
    {
        int indx = t[i];
        for(int j = 0; j < N; j++)
            Av[j] = pc_arr[indx * N + j];
        invAz(N, Av, i, iterator, v_arr, Lj_arr, K);
        for(int j = 0; j < N; j++)
            v_arr[indx * N + j] = Av[j];
        double nv = 0;
        for(int j = 0; j < N; j++)
            nv += v_arr[indx * N + j] * v_arr[indx * N + j];
        Nj_arr[indx] = (sqrt(1 - c1) / nv) *
            (sqrt(1 + (c1 / (1 - c1)) * nv) - 1);
        Lj_arr[indx] = (1 / (sqrt(1 - c1) * nv)) *
            (1 - (1 / sqrt(1 + (c1 / (1 - c1)) * nv)));
    }
    // end of procedure

    if(itr > 0)
    {
        for(int i = 0; i < lambda; i++)
        {
            mixed[i] = arfitness[i];
            mixed[lambda + i] = prev_arfitness[i];
        }
        myqsort(2 * lambda, mixed, ranks, arr_tmp);
        double meanprev = 0;
        double meancur = 0;
        for(int i = 0; i < 2 * lambda; i++)
            ranks_tmp[i] = ranks[i];
        for(int i = 0; i < 2 * lambda; i++)
            ranks[ranks_tmp[i]] = i;
        for(int i = 0; i < lambda; i++)
        {
            meanprev = meanprev + ranks[i];
            meancur = meancur + ranks[lambda + i];
        }
        meanprev = meanprev / lambda;
        meancur = meancur / lambda;
        double diffv = (meancur - meanprev) / lambda;
        double z1 = diffv - val_target;
        s = (1 - cs) * s + cs * z1;
        double d_s = 1;//2.0*(N-1.0)/N;
        sigma = sigma * exp(s / d_s);
    }
    for(int i = 0; i < lambda; i++)
        prev_arfitness[i] = arfitness[i];

    itr++;
}

bool LMCMA::isBehaviorLearningDone()
{
    return CMABase::isBehaviorLearningDone() || sigma < 1e-20;
}

void LMCMA::computeAz(double* Az, double* z)
{
    for(int k = 0; k < N; k++)
        Az[k] = z[k];
    for(int k = 0; k < iterator_sz; k++)
    {
        int jcur = iterator[k];
        double* pc_j = &pc_arr[jcur * N];
        double* v_j = &v_arr[jcur * N];
        double v_j_mult_z = 0;
        for(int p = 0; p < N; p++)
            v_j_mult_z = v_j_mult_z + v_j[p] * z[p];
        v_j_mult_z = Nj_arr[jcur] * v_j_mult_z;
        for(int p = 0; p < N; p++)
            Az[p] = M * Az[p] + v_j_mult_z * pc_j[p];
    }
}

void LMCMA::invAz(int N, double* Av, int iterator_sz, int* iterator,
           double* v_arr, double* Lj_arr, double K)
{
    for(int j = 0; j < iterator_sz; j++)
    {
        int jcur = iterator[j];
        double* v_j = &v_arr[jcur * N];
        double v_j_mult_Av = 0;
        for(int p = 0; p < N; p++)
            v_j_mult_Av += v_j[p] * Av[p];
        v_j_mult_Av = Lj_arr[jcur] * v_j_mult_Av;
        for(int p = 0; p < N; p++)
            Av[p] = K * Av[p] - v_j_mult_Av * v_j[p];
    }
}


SepCMA::SepCMA(double* initialParams, int lambda, double* loBounds,
       double* hiBounds, double sigma, double* covariance, int inseed,
       bool verbose)
    : CMABase(initialParams, lambda, loBounds, hiBounds, covariance, inseed,
              verbose),
      sigma(sigma)
{}

SepCMA::~SepCMA()
{
    delete[] arr_tmp;
    delete[] pc;
    delete[] arz;
    delete[] arindex;
    delete[] diagD;
    delete[] diagC;
    delete[] ps;
}

void SepCMA::init(int N)
{
    CMABase::init(N);

    arz = new double[N*lambda];
    diagD = new double[N];
    diagC = new double[N];
    pc = new double[N];
    ps = new double[N];
    arindex = new int[lambda];
    arr_tmp = new sortedvals[2*lambda];

    for(int i = 0; i < N; i++)
    {
        diagD[i] = 1;
        diagC[i] = 1;
        ps[i] = 0;
        pc[i] = 0;
    }

    cc = 1.0 / lambda;
    c1 = 2.0 / (pow((N + 1.3), 2.0) + mueff);
    cmu =  std::min(1.0 - c1, 2.0 * (mueff - 2.0 + 1.0 / mueff) /
                (pow(N + 2.0, 2.0) + mueff));
    ccov1_sep = std::min(1.0, c1 * (N + 1.5) / 3.0);
    ccovmu_sep = std::min(1.0 - ccov1_sep, cmu * (N + 1.5) / 3);
    chiN = sqrt(double(N)) * (1.0 - 1.0 / (4.0 * N) + 1 / (21.0 * N * N));
    cs = (mueff + 2.0) / (N + mueff + 5.0);
    damps = 1.0 + 2 * std::max(0.0, sqrt((mueff - 1.0) / (N + 1.0)) -
                               1.0) + cs;

    sample();
}

void SepCMA::sample()
{
    for(int i = 0; i < lambda; i++) // O(lambda*m*n)
    {
        sampleStandardNormal(&arz[i * N]);
        for(int k = 0; k < N; k++)  // O(n)
            arx[i * N + k] = xmean[k] + sigma * diagD[k] * arz[i * N + k];
    }
    applyBoundaries();
}

void SepCMA::update()
{
    myqsort(lambda, arfitness, arindex, arr_tmp);
    for(int i = 0; i < N; i++)
    {
        xold[i] = xmean[i];
        xmean[i] = 0;
    }
    for(int i = 0; i < mu; i++)
    {
        double* x = &arx[arindex[i] * N];
        for(int j = 0; j < N; j++)
            xmean[j] += weights[i] * x[j];
    }
    double norm_ps = 0;
    for(int i = 0; i < N; i++)
    {
        ps[i] = (1 - cs) * ps[i] + sqrt(cs * (2 - cs) * mueff) *
            (1. / diagD[i]) * (xmean[i] - xold[i]) / sigma;
        norm_ps += ps[i] * ps[i];
    }
    norm_ps = sqrt(norm_ps);
    for(int i = 0; i < N; i++)
        pc[i] = (1 - cc) * pc[i] + sqrt(cc * (2 - cc) * mueff) *
            (xmean[i] - xold[i]) / sigma;
    for(int i = 0; i < N; i++)
    {
        double val = 0;
        for(int j = 0; j < mu; j++)
            val += weights[j] * arz[arindex[j] * N + i] *
                arz[arindex[j] * N + i];
        diagC[i] = (1 - ccov1_sep - ccovmu_sep) * diagC[i] +
            ccov1_sep * pc[i] * pc[i] + ccovmu_sep * (diagC[i] * val);
    }
    sigma *= exp((cs / damps) * (norm_ps / chiN - 1));
    for(int i = 0; i < N; i++)
        diagD[i] = sqrt(diagC[i]);
    itr++;
}


CMAChol::CMAChol(double* initialParams, int lambda, double* loBounds,
                 double* hiBounds, double sigma, double* covariance,
                 int inseed, bool verbose)
    : CMABase(initialParams, lambda, loBounds, hiBounds, covariance, inseed,
              verbose),
      sigma(sigma)
{}

CMAChol::~CMAChol()
{
    delete[] arr_tmp;
    delete[] pc;
    delete[] arz;
    delete[] arindex;
    delete[] tmp_vec;
    delete[] tmp_vec2;
    delete[] ps;
    delete[] A;
    delete[] Ainv;
    delete[] zmean;
}

void CMAChol::init(int N)
{
    CMABase::init(N);

    A = new double[N*N];
    Ainv = new double[N*N];
    arz = new double[N*lambda];
    pc = new double[N];
    ps = new double[N];
    zmean = new double[N];
    tmp_vec = new double[N];
    tmp_vec2 = new double[N];
    arindex = new int[lambda];
    arr_tmp = new sortedvals[2*lambda];

    double sum_weights = 0;
    for(int i = 0; i < mu; i++)
        sum_weights += log(i + 1.0);
    mueff = 0;
    for(int i = 0; i < mu; i++)
    {
        weights[i] = (log(double(mu + 1.0)) - log(double(1.0 + i))) /
                     (mu * log(mu + 1.0) - sum_weights);
        mueff += weights[i] * weights[i];
    }
    mueff = 1.0 / mueff;

    for(int i = 0; i < N; i++)
    {
        ps[i] = 0;
        pc[i] = 0;
        for(int j = 0; j < N; j++)
            if(i == j)
            {
                A[i * N + j] = 1;
                Ainv[i * N + j] = 1;
            }
            else
            {
                A[i * N + j] = 0;
                Ainv[i * N + j] = 0;
            }
    }

    cc = 4.0 / (N + 4.0);
    c1 = 2.0 / pow(N + sqrt(2.0), 2.0);
    chiN = sqrt(double(N)) * (1.0 - (1.0 / (4.0 * double(N))) +
                              (1.0 / (21.0 * double(N * N))));
    cs = sqrt(mueff) / (sqrt(double(N)) + sqrt(mueff));
    damps = 1.0 + 2.0 * std::max(0.0, sqrt((mueff - 1.0) / (N + 1.0)) -
                                 1.0) + cs;

    sample();
}

void CMAChol::sample()
{
    for(int i = 0; i < lambda; i++) // O(lambda*m*n)
    {
        sampleStandardNormal(&arz[i * N]);
        for(int k = 0; k < N; k++)  // O(n^2)
        {
            double Az = 0;
            double* xcur = &arz[i * N];
            double* Acur = &A[k * N];
            for(int p = 0; p < N; p++)
                Az += Acur[p] * xcur[p];
            arx[i * N + k] = xmean[k] + sigma * Az;
        }
    }
    applyBoundaries();
}

void CMAChol::update()
{
    myqsort(lambda, arfitness, arindex, arr_tmp);
    for(int i = 0; i < N; i++)
    {
        xold[i] = xmean[i];
        xmean[i] = 0;
        zmean[i] = 0;
    }
    for(int i = 0; i < mu; i++)
    {
        double* cur_x = &arx[arindex[i] * N];
        double* cur_z = &arz[arindex[i] * N];
        for(int j = 0; j < N; j++)
        {
            xmean[j] += weights[i] * cur_x[j];
            zmean[j] += weights[i] * cur_z[j];
        }
    }
    double norm_ps = 0;
    for(int i = 0; i < N; i++)
    {
        ps[i] = (1 - cs) * ps[i] + sqrt(cs * (2 - cs) * mueff) * zmean[i];
        norm_ps += ps[i] * ps[i];
    }
    norm_ps = sqrt(norm_ps);
    for(int i = 0; i < N; i++)
        pc[i] = (1 - cc) * pc[i] + sqrt(cc * (2 - cc) * mueff) *
            (xmean[i] - xold[i]) / sigma;
    CholeskyUpdate(pc, Ainv, A);
    sigma *= exp((cs / damps) * (norm_ps / chiN - 1));
    itr++;
}

void CMAChol::CholeskyUpdate(double* pc, double* Ainv, double* A)
{
    double* target;
    double* source1;
    double* source2;
    double* sourceEnd;
    double* targetEnd;

    const double alpha = 1 - c1;
    const double sqrtAlpha = sqrt(alpha);
    const double gamma = c1;

    // v = Ainv * pc
    for(target = tmp_vec, source1 = Ainv, targetEnd = tmp_vec + N,
        sourceEnd = pc + N; target < targetEnd; target++)
    {
        *target = 0.0;
        for(source2 = pc; source2 < sourceEnd; source1++, source2++)
            *target += *source1 * *source2;
    }

    // norm_v2 = norm(v)^2
    double norm_v2 = 0.0;
    for(source1 = tmp_vec, sourceEnd = tmp_vec + N; source1 < sourceEnd;
            source1++)
        norm_v2 += *source1 * *source1;
    const double tmp_root = sqrt(1.0 + (gamma / alpha) * norm_v2);

    // Update A
    const double k1 = sqrtAlpha;
    const double k2 = sqrtAlpha / norm_v2 * (tmp_root - 1.0);
    for(source1 = pc, target = A, sourceEnd = pc + N; source1 < sourceEnd;
            source1++)
        for(source2 = tmp_vec; source2 < tmp_vec + N; target++, source2++)
            *target = k1 * *target + k2 * *source1 * *source2;

    // v' * Ainv
    target = tmp_vec2;
    for(int i = 0; i < N; i++, target++)
    {
        *target = 0.0;
        for(source1 = Ainv + i, source2 = tmp_vec, sourceEnd = tmp_vec + N;
                source2 < sourceEnd; source2++, source1+=N)
            *target += *source2 * *source1;
    }

    // Update Ainv
    const double k3 = 1.0 / sqrtAlpha;
    const double k4 = (1.0 - 1.0 / tmp_root) / (sqrtAlpha * norm_v2);
    for(source1 = tmp_vec, target = Ainv, sourceEnd = tmp_vec + N;
            source1 < sourceEnd; source1++)
        for(source2 = tmp_vec2; source2 < tmp_vec2 + N; target++, source2++)
            *target = k3 * *target - k4 * *source1 * *source2;
}


static const int DIFF_RULE_LENGTH = 7;
static const int TRAJECTORY_PADDING = DIFF_RULE_LENGTH - 1;
static const int NUM_DIFF_RULES = 4;
// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0, 0, 0, 1, 0, 0, 0},                                     // position
    {0, 0, -1, 1, 0, 0, 0},                                    // velocity
    {0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0},      // acceleration
    {0, 1/12.0, -17/12.0, 46/12.0, -46/12.0, 17/12.0, -1/12.0} // jerk
};



void covariance(int num_dims, int num_waypoints, double* covariance)
{
    const double dt = 1.0;
    const int order = 2; // Acceleration

    // Initialize
    int rowLen = num_dims * num_waypoints;
    double diff_matrix[rowLen * rowLen];
    double invCovariance[rowLen * rowLen];
    for (int i=0; i<rowLen; ++i)
        for (int j=0; j<rowLen; ++j)
        {
            diff_matrix[i * rowLen + j] = 0.0;
            invCovariance[i * rowLen + j] = 0.0;
        }

    // Fill i-th dimension
    for(int i=0; i<num_dims; i++)
    {
        double* offset = &diff_matrix[i * rowLen * num_waypoints +
                                      i * num_waypoints];
        differentiationMatrix(num_waypoints, order, dt, offset, rowLen);
    }

    // A * A^T
    for(int i = 0; i < rowLen; i++)
        for(int j = 0; j < rowLen; j++)
            for(int k = 0; k < rowLen; k++)
                invCovariance[i * rowLen + j] += diff_matrix[i * rowLen + k] *
                                                 diff_matrix[k * rowLen + j];

    invert(invCovariance, covariance, rowLen);

    // Scale covariance
    double maxVariance = 0.0;
    for(int i = 0; i < rowLen; i++)
        maxVariance = std::max(maxVariance, covariance[i * rowLen + i]);
    const double scaling = maxVariance * num_waypoints;
    for(int i = 0; i < rowLen; i++)
        for(int j = 0; j < rowLen; j++)
            covariance[i * rowLen + j] /= scaling;
}

void differentiationMatrix(int num_time_steps, int order, double dt,
                           double* diff_matrix, int rowLen)
{
    if(rowLen < 0)
      rowLen = num_time_steps;
    for (int i=0; i<num_time_steps; ++i)
        for (int j=0; j<num_time_steps; ++j)
            diff_matrix[i * rowLen + j] = 0.0;
    double multiplier = 1.0/pow(dt,(int)order);
    for (int i=0; i<num_time_steps; ++i)
    {
        for (int j=-DIFF_RULE_LENGTH/2; j<=DIFF_RULE_LENGTH/2; ++j)
        {
          int index = i+j;
          if (index < 0)
              continue;
          if (index >= num_time_steps)
              continue;
          diff_matrix[i*rowLen + index] += multiplier *
              DIFF_RULES[order][j+DIFF_RULE_LENGTH/2];
        }
    }
}

void invert(double* A, double* Ainv, int N)
{
    Eigen::Map<Eigen::MatrixXd> B(A, N, N);
    Eigen::MatrixXd Binv = B.inverse();
    for(int n = 0; n < N * N; n++)
        Ainv[n] = Binv.data()[n];
}

void cholesky(double* C, double* L, int N)
{
    Eigen::MatrixXd Lm = Eigen::Map<Eigen::MatrixXd>(C, N, N).llt().matrixL();
    double temp;
    for(int n = 0; n < N; n++)
        for(int m = 0; m < N; m++)
        {
            L[m * N + n] = Lm(n, m);
            temp=L[m * N + n];
        }

}

void applyCovL(double* L, double* z, int N)
{
    Eigen::Map<Eigen::MatrixXd> Lm(L, N, N);
    Eigen::Map<Eigen::VectorXd> zv(z, N);
    Eigen::VectorXd znew = Lm * zv;
    for(int n = 0; n < N; n++)
        z[n] = znew(n);
}

