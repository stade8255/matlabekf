function tA = createtA()

global  samplingtime 

%w = sqrt( 980/ 66.6) ;


tA = [ 1         samplingtime     1/2*samplingtime*samplingtime;
       0         1                samplingtime ;
       0         0                1
];