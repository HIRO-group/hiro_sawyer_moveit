// Copyright 2008-2016 Conrad Sanderson (http://conradsanderson.id.au)
// Copyright 2008-2016 National ICT Australia (NICTA)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ------------------------------------------------------------------------


//! \addtogroup Glue
//! @{



//! Class for storing data required for delayed binary operations,
//! such as the operands (e.g. two matrices) and the binary operator (e.g. addition).
//! The operands are stored as references (which can be optimised away),
//! while the operator is "stored" through the template definition (glue_type).
//! The operands can be 'Mat', 'Row', 'Col', 'Op', and 'Glue'.
//! Note that as 'Glue' can be one of the operands, more than two matrices can be stored.
//!
//! For example, we could have: Glue<Mat, Mat, glue_times>
//! 
//! Another example is: Glue< Op<Mat, op_htrans>, Op<Mat, op_inv>, glue_times >



template<typename T1, typename T2, typename glue_type>
class Glue : public Base<typename T1::elem_type, Glue<T1, T2, glue_type> >
  {
  public:
  
  typedef typename T1::elem_type                   elem_type;
  typedef typename get_pod_type<elem_type>::result pod_type;
  
  static const bool is_row  = glue_type::template traits<T1,T2>::is_row;
  static const bool is_col  = glue_type::template traits<T1,T2>::is_col;
  static const bool is_xvec = glue_type::template traits<T1,T2>::is_xvec;
  
  arma_inline  Glue(const T1& in_A, const T2& in_B);
  arma_inline  Glue(const T1& in_A, const T2& in_B, const uword in_aux_uword);
  arma_inline ~Glue();
  
  const T1&   A;          //!< first operand;  must be derived from Base
  const T2&   B;          //!< second operand; must be derived from Base
        uword aux_uword;  //!< storage of auxiliary data, uword format
  };



//! @}
