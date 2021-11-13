%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ~~~                                                                  ~~~ %
% ~~~       USER DEFINED INPUTS-OUTPUTS OF THE EVALUATION MODEL        ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                  Benchmark Control Problem for                   ~~~ %
% ~~~               Seismic Response of Highway Crossing               ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   City College of New York                       ~~~ %
% ~~~                         October, 2004                            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~                   coded by:     Ping TAN                         ~~~ %
% ~~~                 Supervised by:  Prof. Anil K. Agrawal            ~~~ %
% ~~~                                                                  ~~~ %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 

% NOTE: Participants in the benchmark study should change this file
% appropriately to change the inputs and outputs of the evaluation model
% to implement their control design. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      KEY                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Measurement                       Value     %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% rel. displacement                   1       %
% rel. velocity                       2       %
% abs. acceleration                   3       %
% shear force                         4       %
% overtuning moment                   5       %
% curvature                           6       %
% energy                              7       %%                                             %


% Degree of freedom                Value      %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% Translation x                       1       %
% Translation y                       2       %
% Translation z                       3       %
% Rotation x                          4       %
% Rotation y                          5       %
% Rotation z                          6       %


% Element Force Degree of Freedom  Value      %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% Force X first node                  1       %
% Force Y first node                  2       %
% Force Z first node                  3       %
% Moment X first node                 4       %
% Moment Y first node                 5       %
% Moment Z first node                 6       %

% Force X second node                 7       %
% Force Y second node                 8       %
% Force Z second node                 9       %
% Moment X second node               10       %
% Moment Y second node               11       %
% Moment Z second node               12       %

% NOTE: The measures obtained with the above  %
%       degrees of freedom are obtained in    %
%       global coordinates. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% fprintf('User Defined Input-Outputs DATA\n\n')

% Evaluation output
%output#      node#/elem#     dof#         output type
Yen = [
  1              65            1            4                % Base shear
  2              65            2            4
  3              76            1            4
  4              76            2            4
  5              65            1            5                % Overturning moment
  6              65            2            5
  7              76            1            5
  8              76            2            5
  9              16            1            1                % Relative displacement at midspan
 10              16            2            1
 11              16            1            2                % Relative velocity at midspan
 12              16            2            2
 13              16            1            3                % Absolute acceleration at midspan
 14              16            2            3
 15              60            5            6                % Flexural curvature of bent column 1 in local coordinate
 16              60            6            6
 17              61           11            6            
 18              61           12            6
 19              61            5            6            
 20              61            6            6
 21              62           11            6            
 22              62           12            6
 23              62            5            6            
 24              62            6            6
 25              63           11            6            
 26              63           12            6
 27              63            5            6            
 28              63            6            6
 29              64           11            6            
 30              64           12            6
 31              64            5            6            
 32              64            6            6
 33              65           11            6            
 34              65           12            6
 35              71            5            6                % Flexural curvature of bent column 2 in local coordinate
 36              71            6            6
 37              72           11            6            
 38              72           12            6
 39              72            5            6            
 40              72            6            6
 41              73           11            6            
 42              73           12            6
 43              73            5            6            
 44              73            6            6
 45              74           11            6            
 46              74           12            6
 47              74            5            6            
 48              74            6            6
 49              75           11            6            
 50              75           12            6
 51              75            5            6            
 52              75            6            6
 53              76           11            6            
 54              76           12            6
 55             101            1            1                % Bearing deformation
 56             101            2            1
 57             102            1            1            
 58             102            2            1
 59             103            1            1            
 60             103            2            1
 61             104            1            1            
 62             104            2            1
 63             113            1            1            
 64             113            2            1
 65             114            1            1            
 66             114            2            1
 67             115            1            1            
 68             115            2            1
 69             116            1            1            
 70             116            2            1
 71              33            1            1
 72              33            2            1
 73              54            1            1
 74              54            2            1
 75             101            1            4                % Bearing shear force
 76             101            2            4
 77             102            1            4            
 78             102            2            4
 79             103            1            4            
 80             103            2            4
 81             104            1            4            
 82             104            2            4
 83             113            1            4            
 84             113            2            4
 85             114            1            4            
 86             114            2            4
 87             115            1            4            
 88             115            2            4
 89             116            1            4            
 90             116            2            4
 91              33            1            4
 92              33            2            4
 93              54            1            4
 94              54            2            4
 95              60            5            7                % dissipated energy by palstic hinges of bent column 1
 96              60            6            7
 97              61           11            7            
 98              61           12            7
 99              61            5            7            
100              61            6            7
101              62           11            7            
102              62           12            7
103              62            5            7            
104              62            6            7
105              63           11            7            
106              63           12            7
107              63            5            7            
108              63            6            7
109              64           11            7            
110              64           12            7
111              64            5            7            
112              64            6            7
113              65           11            7            
114              65           12            7
115              71            5            7                % dissipated energy by palstic hinges of bent column 2
116              71            6            7
117              72           11            7            
118              72           12            7
119              72            5            7            
120              72            6            7
121              73           11            7            
122              73           12            7
123              73            5            7            
124              73            6            7
125              74           11            7            
126              74           12            7
127              74            5            7            
128              74            6            7
129              75           11            7            
130              75           12            7
131              75            5            7            
132              75            6            7
133              76           11            7            
134              76           12            7
      ];

Oen_bsf = Yen(1:4,1);
Oen_ovm = Yen(5:8,1);
Oen_msd = Yen(9:10,1);
Oen_msv = Yen(11:12,1);
Oen_msa = Yen(13:14,1);
Oen_cur = Yen(15:54,1);
Oen_brd = Yen(55:74,1);
Oen_brf = Yen(75:94,1);
Oen_eng = Yen(95:134,1);

Oen = [1:1:Yen(end,1)];          
Yecm.Oen_bsf = Oen_bsf;
Yecm.Oen_ovm = Oen_ovm;
Yecm.Oen_msd = Oen_msd;
Yecm.Oen_msv = Oen_msv;
Yecm.Oen_msa = Oen_msa;
Yecm.Oen_cur = Oen_cur;
Yecm.Oen_brd = Oen_brd;
Yecm.Oen_brf = Oen_brf;
Yecm.Oen_eng = Oen_eng;
Yecm.Yen = Yen; 
Yecm.Oen = Oen;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Connection vector --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~ %

%output#      node1#       node2#       dof#         output type

Ycn = [       
  1            101          105           1            1               % Relative displacement
  2            101          105           2            1 
  3            102          107           1            1 
  4            102          107           2            1  
  5            103          110           1            1 
  6            103          110           2            1 
  7            104          112           1            1 
  8            104          112           2            1 
  9            113          117           1            1  
 10            113          117           2            1 
 11            114          119           1            1 
 12            114          119           2            1  
 13            115          122           1            1 
 14            115          122           2            1 
 15            116          124           1            1 
 16            116          124           2            1 
 17             33           43           1            1               
 18             33           43           2            1 
 19             54           44           1            1               
 20             54           44           2            1 
 21            101          105           1            2               % Relative velocity
 22            101          105           2            2 
 23            102          107           1            2 
 24            102          107           2            2   
 25            103          110           1            2 
 26            103          110           2            2 
 27            104          112           1            2 
 28            104          112           2            2 
 29            113          117           1            2      
 30            113          117           2            2 
 31            114          119           1            2 
 32            114          119           2            2   
 33            115          122           1            2 
 34            115          122           2            2 
 35            116          124           1            2 
 36            116          124           2            2 
 37             33           43           1            2               
 38             33           43           2            2 
 39             54           44           1            2               
 40             54           44           2            2 
 41            101            0           1            3               % Absolute acceleration
 42            101            0           2            3 
 43            105            0           1            3 
 44            105            0           2            3 
 45            102            0           1            3 
 46            102            0           2            3 
 47            107            0           1            3 
 48            107            0           2            3 
 49            103            0           1            3 
 50            103            0           2            3 
 51            110            0           1            3 
 52            110            0           2            3 
 53            104            0           1            3 
 54            104            0           2            3 
 55            112            0           1            3 
 56            112            0           2            3 
 57            113            0           1            3 
 58            113            0           2            3 
 59            117            0           1            3 
 60            117            0           2            3 
 61            114            0           1            3 
 62            114            0           2            3 
 63            119            0           1            3 
 64            119            0           2            3 
 65            115            0           1            3 
 66            115            0           2            3 
 67            122            0           1            3 
 68            122            0           2            3 
 69            116            0           1            3 
 70            116            0           2            3 
 71            124            0           1            3 
 72            124            0           2            3 
 73             33            0           1            3 
 74             33            0           2            3 
 75             43            0           1            3 
 76             43            0           2            3 
 77             54            0           1            3 
 78             54            0           2            3 
 79             44            0           1            3 
 80             44            0           2            3 
      ];

Ocnd = find(Ycn(:,5)==1);     
Ocnv = find(Ycn(:,5)==2);
Ocna = find(Ycn(:,5)==3);
Ocn  = [1:1:Ycn(end,1)]+length(Oen);          
Yecm.Ycn = Ycn; 
Yecm.Ocn = Ocn;
length(Ocn);
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Measurement vector --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

%output#       node#       dof#        factor       output type

Ymn = [       
  1              1            1           1            1               % Relative displacement between the deck-ends and abutments
  1            125            1          -1            1 
  2              1            2           1            1               
  2            125            2          -1            1 
  3             32            1           1            1               
  3            126            1          -1            1 
  4             32            2           1            1               
  4            126            2          -1            1 
  5             33            1           1            1               
  5             43            1          -1            1 
  6             33            2           1            1               
  6             43            2          -1            1 
  7             54            1           1            1               
  7             44            1          -1            1 
  8             54            2           1            1               
  8             44            2          -1            1 
  9              1            1           1            3               % acceleration at left deck-end 
 10              1            2           1            3 
 11             16            1           1            3               % midspan acceleration
 12             16            2           1            3 
 13             32            1           1            3               % acceleration at right deck-end
 14             32            2           1            3 
 15             33            1           1            3               % acceleration at up beam-end 
 16             54            1           1            3               % acceleration at lower beam-end
      ];

Omnd = [1:8];     
Omna = [9:16];
Omn = [1:1:Ymn(end,1)]+length(Oen)+length(Ocn); 
Oecm = length(Oen)+length(Ocn)+length(Omn);
Yecm.Ymn=Ymn; 
Yecm.Omn=Omn;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %
% ---  Regulated response vector --- %
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %

%output#       node#       dof#        factor       output type

Yrn = [       
  1            101            1           1            1               % Bearing deformation
  1            105            1          -1            1 
  2            101            2           1            1               
  2            105            2          -1            1 
  3            102            1           1            1               
  3            107            1          -1            1 
  4            102            2           1            1               
  4            107            2          -1            1 
  5            103            1           1            1               
  5            110            1          -1            1 
  6            103            2           1            1               
  6            110            2          -1            1 
  7            104            1           1            1               
  7            112            1          -1            1 
  8            104            2           1            1               
  8            112            2          -1            1 
  9            113            1           1            1               
  9            117            1          -1            1 
 10            113            2           1            1               
 10            117            2          -1            1 
 11            114            1           1            1               
 11            119            1          -1            1 
 12            114            2           1            1               
 12            119            2          -1            1  
 13            115            1           1            1               
 13            122            1          -1            1 
 14            115            2           1            1               
 14            122            2          -1            1 
 15            116            1           1            1               
 15            124            1          -1            1 
 16            116            2           1            1               
 16            124            2          -1            1 
 17             33            1           1            1               
 17             43            1          -1            1
 18             33            2           1            1               
 18             43            2          -1            1 
 19             54            1           1            1               
 19             44            1          -1            1 
 20             54            2           1            1               
 20             44            2          -1            1 
 21             16            1           1            1                % Midspan displacement
 22             16            2           1            1 
 23             16            1           1            3                % Midspan acceleration
 24             16            2           1            3 
      ];
     
Orn = [1:1:Yrn(end,1)];          

eval('ctrldev')