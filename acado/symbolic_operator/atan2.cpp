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
*    \file src/symbolic_operator/atan.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO


Atan2::Atan2():BinaryOperator(){ }

Atan2::Atan2( Operator *_argument1, Operator *_argument2 )
        :BinaryOperator( _argument1, _argument2 ){

}


Atan2::Atan2( const Atan2 &arg ):BinaryOperator( arg ){

}


Atan2::~Atan2(){

}

Atan2& Atan2::operator=( const Atan2 &arg ){

    if( this != &arg ){

        BinaryOperator::operator=( arg );
    }

    return *this;
}



returnValue Atan2::evaluate( int number, double *x, double *result ){

    if( number >= bufferSize ){
        bufferSize += number;
        argument1_result  = (double*)realloc( argument1_result,bufferSize*sizeof(double));
        argument2_result  = (double*)realloc( argument2_result,bufferSize*sizeof(double));
        dargument1_result = (double*)realloc(dargument1_result,bufferSize*sizeof(double));
        dargument2_result = (double*)realloc(dargument2_result,bufferSize*sizeof(double));
    }

    argument1->evaluate( number, x , &argument1_result[number] );
    argument2->evaluate( number, x , &argument2_result[number] );

    result[0] = atan2(argument1_result[number],argument2_result[number]);

    return SUCCESSFUL_RETURN;
}


/*returnValue Atan2::evaluate( EvaluationBase *x ){
 
    x->product(*argument1,*argument2);
    return SUCCESSFUL_RETURN;
}*/

std::ostream& Atan2::print( std::ostream &stream ) const{

	if ( ( acadoIsFinite( argument1->getValue() ) == BT_FALSE ) ||
		 ( acadoIsFinite( argument2->getValue() ) == BT_FALSE ) )
	{
		return stream << "atan2(" << *argument1 << "," << *argument2 << ")";
	}
	else
	{
		return stream << "((real_t)(" << atan2(argument1->getValue(),argument2->getValue()) << "))";
	}
}


/*Operator* Product::differentiate( int index ){

	dargument1 = argument1->differentiate( index );
	dargument2 = argument2->differentiate( index );

	Operator *prodTmp1 = myProd(dargument1, argument2);
	Operator *prodTmp2 = myProd(argument1, dargument2);
	Operator *result = myAdd(prodTmp1, prodTmp2);

	delete prodTmp1;
	delete prodTmp2;

	return result;
}*/


Operator* Atan2::AD_forward( int dim,
                                 VariableType *varType,
                                 int *component,
                                 Operator **seed,
                                 int &nNewIS,
                                 TreeProjection ***newIS ){

    if( dargument1 != 0 )
        delete dargument1;

    if( dargument2 != 0 )
        delete dargument2;

    dargument1 = argument1->AD_forward(dim,varType,component,seed,nNewIS,newIS);
    dargument2 = argument2->AD_forward(dim,varType,component,seed,nNewIS,newIS);

    Operator *prodTmp1 = myProd(dargument1, argument2);
    Operator *prodTmp2 = myProd(argument1, dargument2);
    Operator *result = myAdd(prodTmp1, prodTmp2);

    delete prodTmp1;
    delete prodTmp2;

    return result;
}


CLOSE_NAMESPACE_ACADO

// end of file.
