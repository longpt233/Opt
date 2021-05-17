package optimal_schedular;

import java.io.File;
import java.util.*;

import localsearch.domainspecific.vehiclerouting.vrp.ConstraintSystemVR;
import localsearch.domainspecific.vehiclerouting.vrp.IFunctionVR;
import localsearch.domainspecific.vehiclerouting.vrp.VRManager;
import localsearch.domainspecific.vehiclerouting.vrp.VarRoutesVR;
import localsearch.domainspecific.vehiclerouting.vrp.constraints.eq.Eq;
import localsearch.domainspecific.vehiclerouting.vrp.constraints.leq.Leq;
import localsearch.domainspecific.vehiclerouting.vrp.entities.ArcWeightsManager;
import localsearch.domainspecific.vehiclerouting.vrp.entities.NodeWeightsManager;
import localsearch.domainspecific.vehiclerouting.vrp.entities.Point;
import localsearch.domainspecific.vehiclerouting.vrp.functions.AccumulatedEdgeWeightsOnPathVR;
import localsearch.domainspecific.vehiclerouting.vrp.functions.AccumulatedNodeWeightsOnPathVR;
import localsearch.domainspecific.vehiclerouting.vrp.functions.IndexOnRoute;
import localsearch.domainspecific.vehiclerouting.vrp.functions.LexMultiFunctions;
import localsearch.domainspecific.vehiclerouting.vrp.functions.RouteIndex;
import localsearch.domainspecific.vehiclerouting.vrp.functions.TotalCostVR;
import localsearch.domainspecific.vehiclerouting.vrp.invariants.AccumulatedWeightEdgesVR;
import localsearch.domainspecific.vehiclerouting.vrp.invariants.AccumulatedWeightNodesVR;

public class HeuristicOnePoint {
    int N, M, K; // N khach M goi hang
    int[] q; // khoi luong hang tai diem
    int[] Q; // khoi luong max cua xe
    int[][] d; // distance

    ArrayList<Point> start;
    ArrayList<Point> end;
    ArrayList<Point> clPoint; // tat ca point nhung trong so khac
    ArrayList<Point> pkPoint;
    ArrayList<Point> clientPoints;
    ArrayList<Point> personPoints;
    ArrayList<Point> packPoints;
    ArrayList<Point> personOutPoints;
    ArrayList<Point> packOutPoints;
    ArrayList<Point> allPersonPoints;
    ArrayList<Point> allPackPoints;
    ArrayList<Point> servicePoints; // tong cac diem can phuc vu N+M
    ArrayList<Point> allPoints; // tinh ca 2 diem gia

    NodeWeightsManager clMng;
    NodeWeightsManager pkMng;
    ArcWeightsManager weightsMng; // luu tru trong so tren canh noi giua cac point

    HashMap<Point, Integer> mapPoint2ID;
    HashMap<Integer, Point> reverseMapPoint2ID;

    // modelling
    VRManager mgr;
    VarRoutesVR XR; // bien loi giai (luu tap cac route)
    ConstraintSystemVR CS;
    LexMultiFunctions F;
    IFunctionVR obj;
    IFunctionVR[] d2; // danh co thiet lap rang buoc cho tat ca cac diem
    IFunctionVR[] d3; // danh co thiet lap rang buoc ve truoc sau
    IFunctionVR[] d4; // danh co thiet lap rang buoc dung diem lay va tra
    IFunctionVR[] cost; // cost[k] la chieu dai cua route thu k
    Random R = new Random();

    public void readData(String fn) {
        try {
            Scanner in = new Scanner(new File(fn));
            N = in.nextInt();
            M = in.nextInt();
            K = in.nextInt();

            q = new int[M + 1];
            Q = new int[K + 1];
            d = new int[2 * N + 2 * M + 1][2 * N + 2 * M + 1];

            for (int i = 1; i <= M; i++) {
                q[i] = in.nextInt();
            }
            for (int i = 1; i <= K; i++) {
                Q[i] = in.nextInt();
            }
            for (int i = 0; i <= 2 * N + 2 * M; i++) {
                for (int j = 0; j <= 2 * N + 2 * M; j++) {
                    d[i][j] = in.nextInt();
                }
            }
            in.close();
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void printReadData() {
        System.out.println(N + " " + M + " " + K);
        for (int i = 1; i <= M; i++) {
            System.out.print(q[i] + " ");
        }
        System.out.println();
        for (int i = 1; i <= K; i++) {
            System.out.print(Q[i] + " ");
        }
        System.out.println();
        for (int i = 0; i <= 2 * N + 2 * M; i++) {
            for (int j = 0; j <= 2 * N + 2 * M; j++) {
                System.out.print(d[i][j] + " ");
            }
            System.out.println();
        }
    }

    public void mapping() {
        start = new ArrayList<Point>();
        end = new ArrayList<Point>();
        clPoint = new ArrayList<Point>();
        pkPoint = new ArrayList<Point>();
        clientPoints = new ArrayList<Point>();
        personPoints = new ArrayList<Point>();
        packPoints = new ArrayList<Point>();
        personOutPoints = new ArrayList<Point>();
        packOutPoints = new ArrayList<Point>();
        servicePoints = new ArrayList<Point>();
        allPoints = new ArrayList<Point>();
        mapPoint2ID = new HashMap<Point, Integer>();
        reverseMapPoint2ID = new HashMap< Integer,Point>();
        allPersonPoints =new ArrayList<Point>();
        allPackPoints =new ArrayList<Point>();

        // khoi tao cac diem bat dau va ket thuc cua cac xe (route)
        for (int k = 1; k <= K; k++) {
            // them 2 k diem gia
            Point s = new Point(0);
            Point t = new Point(0);
            start.add(s);
            end.add(t);
            allPoints.add(s);
            allPoints.add(t);

            mapPoint2ID.put(s, 0);
            mapPoint2ID.put(t, 0);
            reverseMapPoint2ID.put(0, s);
            reverseMapPoint2ID.put(0, t);
        }

        // client Point
        for (int i = 1; i <= N; i++) {
            Point p = new Point(i);
            personPoints.add(p);
            allPoints.add(p);
            servicePoints.add(p);
            mapPoint2ID.put(p, i);
            reverseMapPoint2ID.put(i,p);
        }
        // pack point
        for (int i = N + 1; i <= N + M; i++) {
            Point p = new Point(i);
            packPoints.add(p);
            allPoints.add(p);
            servicePoints.add(p);
            mapPoint2ID.put(p, i);
            reverseMapPoint2ID.put(i,p);
        }
        // them cac diem tra khach
        for (int i = N + M + 1; i <= 2 * N + M; i++) {
            Point p = new Point(i);
            personOutPoints.add(p);
            allPoints.add(p);
            mapPoint2ID.put(p, i);
            reverseMapPoint2ID.put(i,p);
        }
        // diem tra hang
        for (int i = 2 * N + M + 1; i <= 2 * N + 2 * M; i++) {
            Point p = new Point(i);
            packOutPoints.add(p);
            allPoints.add(p);
            mapPoint2ID.put(p, i);
            reverseMapPoint2ID.put(i,p);
        }

        allPersonPoints.addAll(personPoints);
        allPersonPoints.addAll(personOutPoints);
        allPackPoints.addAll(packPoints);
        allPackPoints.addAll(packOutPoints);

        clientPoints.addAll(allPersonPoints);
        clientPoints.addAll(allPackPoints);

        clPoint.addAll(allPoints);
        pkPoint.addAll(allPoints);

        clMng = new NodeWeightsManager(clPoint);
        pkMng = new NodeWeightsManager(pkPoint);
        weightsMng = new ArcWeightsManager(allPoints);

        // set trong so duong di
        for (Point p : allPoints){
            for (Point q : allPoints) {
                int ip = mapPoint2ID.get(p);
                int iq = mapPoint2ID.get(q);
                weightsMng.setWeight(p, q, d[ip][iq]);
            }
        }

        // set trong so tai moi node (node person hoac pack)
        // rang buoc tra hang voi khach thi trong so am di, khong co lien quan thi trong so = 0
        for (Point p : allPoints){
            clMng.setWeight(p, 0);
            pkMng.setWeight(p, 0);
            int tmp = mapPoint2ID.get(p);
            if(tmp == 0 ) continue;
            if (tmp <= N) {
                clMng.setWeight(p, 1);
            } else if (tmp <= N + M) {
                pkMng.setWeight(p, q[tmp-N]);
            } else if (tmp <= 2 * N + M) {
                clMng.setWeight(p, -1);
            } else if (tmp <= 2 * N + 2 * M) {
                pkMng.setWeight(p, -q[tmp-N-M-N]);
            }
        }
    }

    public void stateModel2(){
        mgr = new VRManager();
        XR = new VarRoutesVR(mgr);
        // cac phuong an se dc dem tu 0
        for (int i = 0; i < start.size(); i++) {
            Point s = start.get(i);
            Point t = end.get(i);
            XR.addRoute(s, t);
        }
        // add co the di qua
        for (Point p : allPersonPoints) {
            XR.addClientPoint(p);// khai bao XR co the se di qua diem p
        }
        for (Point p : allPackPoints) {
            XR.addClientPoint(p);// co the o day da xet la khogn lap lai roi
        }
        // thiet lap rang buoc
        CS = new ConstraintSystemVR(mgr);

        AccumulatedWeightNodesVR personAccum = new AccumulatedWeightNodesVR(XR, clMng);
        AccumulatedWeightNodesVR packAccum = new AccumulatedWeightNodesVR(XR, pkMng);
        AccumulatedWeightEdgesVR weightAccum = new AccumulatedWeightEdgesVR(XR, weightsMng);

//        d1 = new IFunctionVR[2 * N + 2 * M + 2 * K + 2];

        d2 = new IFunctionVR[2 * N + 2 * M + 2 * K + 2];
        for (Point p : allPoints) {
            int tmp = mapPoint2ID.get(p);
            if(tmp == 0) continue;
            // rang buoc tai trong
            d2[tmp] = new AccumulatedNodeWeightsOnPathVR(packAccum, p);

            // trong luong toi da cua xe ma diem p thuoc ve
//            int qi= Q[XR.route(p)+1];
//            CS.post(new Leq(d2[tmp], qi));
            CS.post(new Leq(d2[tmp], 10));
            CS.post(new Leq(0, d2[tmp]));

            // rang buoc so nguoi
            d2[tmp] = new AccumulatedNodeWeightsOnPathVR(personAccum, p);
            CS.post(new Leq(d2[tmp], 1));
            CS.post(new Leq(0, d2[tmp]));
        }

        // xet tat ca cac diem thi phai co diem nhan roi thi moi duoc tra
        d3 = new IFunctionVR[2 * N + 2 * M + 2 * K + 10];
        for (Point p : allPoints) {
            int tmp = mapPoint2ID.get(p);
            if (tmp == 0)
                continue;
            if (tmp <= N + M) { // thay vi dung cai nay co the dung service point
                Point pTo = reverseMapPoint2ID.get(tmp + N + M);
                d3[tmp] = new IndexOnRoute(XR, p);
                d3[tmp + N + M] = new IndexOnRoute(XR, pTo);
                CS.post(new Leq(d3[tmp], d3[tmp + N + M]));
            }
        }

        // rang buoc phai trả dung diem len va xuong
        d4 = new IFunctionVR[2 * N + 2 * M + 2 * K + 10];
        for (Point p : allPoints) {
            int tmp = mapPoint2ID.get(p);
            if (tmp == 0)
                continue;
            if (tmp <= N + M) {
                Point pTo = reverseMapPoint2ID.get(tmp + N + M);
                d4[tmp] = new RouteIndex(XR, p);
                d4[tmp + N + M] = new RouteIndex(XR, pTo);
                CS.post(new Eq(d4[tmp], d4[tmp + N + M]));
            }
        }

        cost = new IFunctionVR[K];
        for (int k = 1; k <= K; k++) {
            Point tk = XR.endPoint(k);
            cost[k - 1] = new AccumulatedEdgeWeightsOnPathVR(weightAccum, tk);
        }

        obj = new TotalCostVR(XR, weightsMng);// tong khoang cach di chuyen cua K xe (route)
        mgr.close();
    }

    public void initialSolution() {
        ArrayList<Point> listPoints = new ArrayList<Point>();
        for (int k = 1; k <= XR.getNbRoutes(); k++) {
            listPoints.add(XR.startPoint(k));
        }
        for (Point p : allPersonPoints) {
            Point x = listPoints.get(R.nextInt(listPoints.size()));
            mgr.performAddOnePoint(p, x);
            System.out.println(XR.toString() + "violations = " + CS.violations() + ", cost = " + obj.getValue());
            listPoints.add(p);
        }
        for (Point p : allPackPoints) {
            Point x = listPoints.get(R.nextInt(listPoints.size()));
            mgr.performAddOnePoint(p, x);
            System.out.println(XR.toString() + "violations = " + CS.violations() + ", cost = " + obj.getValue());
            listPoints.add(p);
        }
    }

    class Move {
        Point x;
        Point y;
        public Move(Point x, Point y) {
            this.x = x;
            this.y = y;
        }
    }

    public void exploreNeighborhood(ArrayList<Move> cand) {
        cand.clear();
        int minDeltaC = Integer.MAX_VALUE;
        double minDeltaF = minDeltaC;
        for (int k = 1; k <= XR.getNbRoutes(); k++) {
            for (Point y = XR.next(XR.startPoint(k)); y != XR.endPoint(k); y = XR.next(y)) {
                for (Point x : clientPoints)
                    if (x != y && x != XR.next(y)) {
                        int deltaC = CS.evaluateOnePointMove(x, y);
                        double deltaF = obj.evaluateOnePointMove(x, y);
//                        System.out.print(deltaC + "_" + (int)deltaF + " |");
                        if (!(deltaC < 0 || (deltaC == 0 && deltaF < 0)))
                            continue;
                        if (deltaC < minDeltaC || (deltaC == minDeltaC && deltaF < minDeltaF)) {
                            cand.clear();
                            cand.add(new Move(x, y));
                            minDeltaC = deltaC;
                            minDeltaF = deltaF;
                        } else if (deltaC == minDeltaC && deltaF == minDeltaF)
                            cand.add(new Move(x, y));
                    }
            }
        }
    }

    public double search(int maxIter) {
        initialSolution();
        System.out.println("\n");
        int it = 0;
        ArrayList<Move> cand = new ArrayList<Move>();
        while (it < maxIter) {
            exploreNeighborhood(cand);
            if (cand.size() <= 0) {
                System.out.println("Reach local optimum");
                break;
            }
            Move m = cand.get(R.nextInt(cand.size()));
            mgr.performOnePointMove(m.x, m.y);
            System.out.println("\nStep " + it + ", XR   " + m.x.getID() + ","+m.y.getID() +"\n" + XR.toString() + "violations = " + CS.violations()
                    + ", cost = " + obj.getValue()+ "");
            it++;
        }
        return obj.getValue() * (1-Math.signum(CS.violations()));
    }

    public static void testN(HeuristicOnePoint A, int numberTry){
        double best = Integer.MAX_VALUE;
//        int numberTry = new Scanner(System.in).nextInt();
        for (int i = 0; i < numberTry; i++) {
            A.mapping();
            A.stateModel2();
            double cost = A.search(100);
            if (cost == 0) continue;
            System.out.println();
            if (best > cost)
                best = cost;
        }
        System.out.println("\nbest : " + best);
    }


    public static void main(String[] args) {
        HeuristicOnePoint A = new HeuristicOnePoint();

        A.readData("src/optimal_schedular/data.txt");
        A.printReadData();
        testN(A, 10);

//        A.mapping();
//        A.stateModel2();
//        A.search(100);
    }
}