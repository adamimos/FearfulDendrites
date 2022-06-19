/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
#include <stdio.h>
#include <math.h>
#include "scoplib.h"
#undef PI
 
#include "md1redef.h"
#include "section.h"
#include "md2redef.h"

#if METHOD3
extern int _method3;
#endif

#undef exp
#define exp hoc_Exp
extern double hoc_Exp();
 
#define _threadargscomma_ /**/
#define _threadargs_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gnmdamax _p[0]
#define gampamax _p[1]
#define e _p[2]
#define Voff _p[3]
#define Vset _p[4]
#define decayampa _p[5]
#define decaynmda _p[6]
#define xloc _p[7]
#define yloc _p[8]
#define tag1 _p[9]
#define tag2 _p[10]
#define inmda _p[11]
#define iampa _p[12]
#define gnmda _p[13]
#define local_v _p[14]
#define local_ca _p[15]
#define A _p[16]
#define B _p[17]
#define gampa _p[18]
#define dampa _p[19]
#define dnmda _p[20]
#define ica _p[21]
#define cai _p[22]
#define DA _p[23]
#define DB _p[24]
#define Dgampa _p[25]
#define Ddampa _p[26]
#define Ddnmda _p[27]
#define _g _p[28]
#define _tsav _p[29]
#define _nd_area  *_ppvar[0]._pval
#define _ion_cai	*_ppvar[2]._pval
#define _ion_ica	*_ppvar[3]._pval
#define _ion_dicadv	*_ppvar[4]._pval
#define diam	*_ppvar[5]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern int nrn_get_mechtype();
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 static _hoc_setdata(_vptr) void* _vptr; { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 /* connect user functions to hoc names */
 static IntFunc hoc_intfunc[] = {
 0,0
};
 static struct Member_func {
	char* _name; double (*_member)();} _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 0, 0
};
 /* declare global and static user variables */
#define gama gama_glutamate1
 double gama = 0.08;
#define icaconst icaconst_glutamate1
 double icaconst = 0.1;
#define n n_glutamate1
 double n = 0.25;
#define tau2 tau2_glutamate1
 double tau2 = 2;
#define tau1 tau1_glutamate1
 double tau1 = 50;
#define taudnmda taudnmda_glutamate1
 double taudnmda = 200;
#define taudampa taudampa_glutamate1
 double taudampa = 200;
#define tau_ampa tau_ampa_glutamate1
 double tau_ampa = 2;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau1_glutamate1", "ms",
 "tau2_glutamate1", "ms",
 "tau_ampa_glutamate1", "ms",
 "n_glutamate1", "/mM",
 "gama_glutamate1", "/mV",
 "taudampa_glutamate1", "ms",
 "taudnmda_glutamate1", "ms",
 "gnmdamax", "nS",
 "gampamax", "nS",
 "e", "mV",
 "A", "nS",
 "B", "nS",
 "gampa", "nS",
 "inmda", "nA",
 "iampa", "nA",
 "gnmda", "nS",
 "local_v", "mV",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double dnmda0 = 0;
 static double dampa0 = 0;
 static double gampa0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "icaconst_glutamate1", &icaconst_glutamate1,
 "tau1_glutamate1", &tau1_glutamate1,
 "tau2_glutamate1", &tau2_glutamate1,
 "tau_ampa_glutamate1", &tau_ampa_glutamate1,
 "n_glutamate1", &n_glutamate1,
 "gama_glutamate1", &gama_glutamate1,
 "taudampa_glutamate1", &taudampa_glutamate1,
 "taudnmda_glutamate1", &taudnmda_glutamate1,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(), nrn_init(), nrn_state();
 static void nrn_cur(), nrn_jacob();
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(), _ode_map(), _ode_spec(), _ode_matsol();
 
#define _cvode_ieq _ppvar[6]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static char *_mechanism[] = {
 "6.2.0",
"glutamate1",
 "gnmdamax",
 "gampamax",
 "e",
 "Voff",
 "Vset",
 "decayampa",
 "decaynmda",
 "xloc",
 "yloc",
 "tag1",
 "tag2",
 0,
 "inmda",
 "iampa",
 "gnmda",
 "local_v",
 "local_ca",
 0,
 "A",
 "B",
 "gampa",
 "dampa",
 "dnmda",
 0,
 0};
 static Symbol* _morphology_sym;
 static Symbol* _ca_sym;
 
static void nrn_alloc(_prop)
	Prop *_prop;
{
	Prop *prop_ion, *need_memb();
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 30, _prop);
 	/*initialize range parameters*/
 	gnmdamax = 1;
 	gampamax = 1;
 	e = 0;
 	Voff = 0;
 	Vset = -60;
 	decayampa = 0.5;
 	decaynmda = 0.5;
 	xloc = 0;
 	yloc = 0;
 	tag1 = 0;
 	tag2 = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 30;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[5]._pval = &prop_ion->param[0]; /* diam */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static _net_receive();
 typedef (*_Pfrv)();
 extern _Pfrv* pnt_receive;
 extern short* pnt_receive_size;
 static void _update_ion_pointer(Datum*);
 _glutamate1_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func,
	 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_dparam_size(_mechtype, 7);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 glutamate1 /Users/adamimos/LarkumLab/Lucy L23 Project/Basic Setup 53/umac/glutamate1.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double F = 96480.0;
 static double R = 8.314;
 static double PI = 3.14159;
static int _reset;
static char *modelname = "NMDA synapse with depression";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(), _ode_matsol1();
 static int _slist1[5], _dlist1[5];
 static int state();
 extern int state_discon_flag_;
 
static _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   state_discontinuity ( _cvode_ieq + 0, & A , A + gnmdamax * ( dnmda ) ) ;
   state_discontinuity ( _cvode_ieq + 1, & B , B + gnmdamax * ( dnmda ) ) ;
   state_discontinuity ( _cvode_ieq + 2, & gampa , gampa + gampamax * dampa ) ;
   state_discontinuity ( _cvode_ieq + 3, & dampa , dampa * decayampa ) ;
   state_discontinuity ( _cvode_ieq + 4, & dnmda , dnmda * decaynmda ) ;
   } }
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / tau1 ;
   DB = - B / tau2 ;
   Dgampa = - gampa / tau_ampa ;
   Ddampa = ( 1.0 - dampa ) / taudampa ;
   Ddnmda = ( 1.0 - dnmda ) / taudnmda ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( - 1.0 ) / tau1 )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / tau2 )) ;
 Dgampa = Dgampa  / (1. - dt*( ( - 1.0 ) / tau_ampa )) ;
 Ddampa = Ddampa  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taudampa )) ;
 Ddnmda = Ddnmda  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taudnmda )) ;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( - 1.0 ) / tau1)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / tau2)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - B) ;
    gampa = gampa + (1. - exp(dt*(( - 1.0 ) / tau_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_ampa ) - gampa) ;
    dampa = dampa + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taudampa)))*(- ( ( ( 1.0 ) ) / taudampa ) / ( ( ( ( - 1.0) ) ) / taudampa ) - dampa) ;
    dnmda = dnmda + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taudnmda)))*(- ( ( ( 1.0 ) ) / taudnmda ) / ( ( ( ( - 1.0) ) ) / taudnmda ) - dnmda) ;
   }
  return 0;
}
 
static int _ode_count(_type) int _type;{ return 5;}
 
static int _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
     _ode_spec1 ();
  }}
 
static int _ode_map(_ieq, _pv, _pvdot, _pp, _ppd, _atol, _type) int _ieq, _type; double** _pv, **_pvdot, *_pp, *_atol; Datum* _ppd; { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 5; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static int _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
 _ode_matsol1 ();
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 4, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  B = B0;
  dnmda = dnmda0;
  dampa = dampa0;
  gampa = gampa0;
 {
   gnmda = 0.0 ;
   gampa = 0.0 ;
   A = 0.0 ;
   B = 0.0 ;
   dampa = 1.0 ;
   dnmda = 1.0 ;
   ica = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   double _lcount ;
 local_v = v * ( 1.0 - Voff ) + Vset * Voff ;
   gnmda = ( A - B ) / ( 1.0 + n * exp ( - gama * local_v ) ) ;
   inmda = ( 1e-3 ) * gnmda * ( v - e ) ;
   iampa = ( 1e-3 ) * gampa * ( v - e ) ;
   local_v = v ;
   local_ca = cai ;
   ica = inmda * 0.1 / ( PI * diam ) * icaconst ;
   inmda = inmda * .9 ;
   }
 _current += inmda;
 _current += iampa;
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  cai = _ion_cai;
 _g = _nrn_current(_v + .001);
 	{ double _dica;
  _dica = ica;
 state_discon_flag_ = 1; _rhs = _nrn_current(_v); state_discon_flag_ = 0;
  _ion_dicadv += (_dica - ica)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
 double _break, _save;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _break = t + .5*dt; _save = t;
 v=_v;
{
  cai = _ion_cai;
 { {
 for (; t < _break; t += dt) {
 error =  state();
 if(error){fprintf(stderr,"at line 96 in file glutamate1.mod:\n	SOLVE state METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 
}}
 t = _save;
 } }}

}

static terminal(){}

static _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(B) - _p;  _dlist1[1] = &(DB) - _p;
 _slist1[2] = &(gampa) - _p;  _dlist1[2] = &(Dgampa) - _p;
 _slist1[3] = &(dampa) - _p;  _dlist1[3] = &(Ddampa) - _p;
 _slist1[4] = &(dnmda) - _p;  _dlist1[4] = &(Ddnmda) - _p;
_first = 0;
}
