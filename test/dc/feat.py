import json,numpy as np,sys
D=np.array(json.load(open(sys.argv[1])))
# columns: 0 β 1 r 2 v 3 e 4 h 5 δ 6 p 7 thr 8 δ̇ 9 hand 10-13 wheel surface speeds 14 we 15-18 Fz 19 dComp 20 ax 21 ay 22 cmd
X=D[:-1];Y=D[1:]
names={0:'β',1:'r',2:'v',5:'δ',6:'p',7:'thr',8:'δ̇',9:'hand',14:'we',19:'dComp',20:'ax',21:'ay'}
def r2(target,cols):
    A=np.column_stack([X[:,cols],np.ones(len(X))]);y=Y[:,target]-X[:,target]
    w,*_=np.linalg.lstsq(A,y,rcond=None);res=y-A@w;return 1-res.var()/y.var()
v=X[:,2:3]
# wheel slip features
wf=(X[:,10]+X[:,11])/2/X[:,2]-1; wr=(X[:,12]+X[:,13])/2/X[:,2]-1
X=np.column_stack([X,wf,wr])  # 23 wf 24 wr
base=[0,1,2,5,7]
sets={'base β r v δ thr':base,'+dComp':base+[19],'+δ̇':base+[8],'+wheels':base+[23,24],'+we':base+[14],'+Fz':base+[15,16,17,18],'+ax ay':base+[20,21],'all':base+[8,19,23,24,14,15,16,17,18,20,21]}
for t,tn in [(0,'β'),(1,'r'),(2,'v')]:
    print(tn,' | '.join(f'{k} {r2(t,c):.2f}' for k,c in sets.items()))
print('δ row', ' | '.join(f'{k} {r2(5,c):.2f}' for k,c in {'δ p':[5,6],'δ p β r v':[5,6,0,1,2],'+δ̇':[5,6,0,1,2,8],'+δ̇ hand':[5,6,0,1,2,8,9],'+cmd':[5,6,0,1,2,8,9,22]}.items()))
Fz=X[:,15:19];sm=Fz.sum(1)
LT=(Fz[:,0]+Fz[:,2]-Fz[:,1]-Fz[:,3])/sm; LO=(Fz[:,0]+Fz[:,1]-Fz[:,2]-Fz[:,3])/sm; LF=(Fz[:,0]-Fz[:,1])/sm; LR=(Fz[:,2]-Fz[:,3])/sm
Fz1=Y[:,15:19];  # not usable as regressor (future) – rates from the previous frame instead
dLT=np.r_[0,np.diff(LT)]; dLO=np.r_[0,np.diff(LO)]
X=np.column_stack([X,LT,LO,LF,LR,dLT,dLO,sm])  # 25 LT 26 LO 27 LF 28 LR 29 dLT 30 dLO 31 sum
sets={'base':base,'+LT LO':base+[25,26],'+LF LR LO':base+[27,28,26],'+LT LO dLT dLO':base+[25,26,29,30],'+LF LR LO dLT dLO':base+[27,28,26,29,30],'+LF LR LO dLT dLO sum dComp':base+[27,28,26,29,30,31,19],'+Fz4':base+[15,16,17,18]}
print('--- load-transfer features')
for t,tn in [(0,'β'),(1,'r'),(2,'v')]:
    print(tn,' | '.join(f'{k} {r2(t,c):.2f}' for k,c in sets.items()))
print('--- inputs: cmd vs p vs δ')
for t,tn in [(0,'β'),(1,'r'),(2,'v')]:
    print(tn,' | '.join(f'{k} {r2(t,c):.2f}' for k,c in {'β r v δ thr':[0,1,2,5,7],'β r v cmd thr':[0,1,2,22,7],'β r v δ cmd thr':[0,1,2,5,22,7],'β r v δ p thr':[0,1,2,5,6,7],'β r v δ δ̇ cmd thr':[0,1,2,5,8,22,7]}.items()))
