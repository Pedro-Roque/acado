/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



/**
 *    \file include/acado/symbolic_operator/Atan2.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#ifndef ACADO_TOOLKIT_Atan2_HPP
#define ACADO_TOOLKIT_Atan2_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the scalar inverse tangens operator (arctan) within the symbolic operators family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Atan2 implements the scalar inverse tanges operator (arctan) within the 
 *	symbolic operators family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Atan2 : public BinaryOperator{

public:

    /** Default constructor. */
    Atan2();

    /** Default constructor. */
    Atan2( Operator *_argument1, Operator *_argument2);

    /** Copy constructor (deep copy). */
    Atan2( const Atan2 &arg );

    /** Default destructor. */
    ~Atan2();

    /** Assignment Operator (deep copy). */
    Atan2& operator=( const Atan2 &arg );

	
	/** Evaluates the expression (templated version) */
	virtual returnValue evaluate( EvaluationBase *x );

	 
    /* Prints the expression with std stream */
    virtual std::ostream& print( std::ostream &stream );


     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     Operator* AD_forward( int dim,
                                 VariableType *varType,
                                 int *component,
                                 Operator **seed,
                                 int &nNewIS,
                                 TreeProjection ***newIS );

     virtual returnValue initDerivative();


//
//  PROTECTED FUNCTIONS:
//

protected:
};


CLOSE_NAMESPACE_ACADO



#endif
