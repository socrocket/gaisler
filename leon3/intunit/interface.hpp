/***************************************************************************\
 *
 *
 *         _/        _/_/_/_/    _/_/    _/      _/   _/_/_/
 *        _/        _/        _/    _/  _/_/    _/         _/
 *       _/        _/_/_/    _/    _/  _/  _/  _/     _/_/
 *      _/        _/        _/    _/  _/    _/_/         _/
 *     _/_/_/_/  _/_/_/_/    _/_/    _/      _/   _/_/_/
 *
 *
 *
 *
 *   This file is part of LEON3.
 *
 *   LEON3 is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the
 *   Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *   or see <http://www.gnu.org/licenses/>.
 *
 *
 *
 *   (c) Luca Fossati, fossati.l@gmail.com
 *
\***************************************************************************/


#ifndef LT_INTERFACE_HPP
#define LT_INTERFACE_HPP

#include "core/trapgen/modules/abi_if.hpp"
#include "gaisler/leon3/intunit/memory.hpp"
#include "gaisler/leon3/intunit/registers.hpp"
#include "gaisler/leon3/intunit/alias.hpp"
#include <boost/circular_buffer.hpp>
#include "core/trapgen/modules/instruction.hpp"
#include <vector>
#include <string>
#include "core/base/systemc.h"

#define FUNC_MODEL
#define LT_IF
using namespace trap;
namespace leon3_funclt_trap{

    class LEON3_ABIIf : public ABIIf< unsigned int >{
        private:
        unsigned int & PROGRAM_LIMIT;
        MemoryInterface & dataMem;
        Reg32_0 & PSR;
        Reg32_1 & WIM;
        Reg32_2 & TBR;
        Reg32_3 & Y;
        Reg32_3 & PC;
        Reg32_3 & NPC;
        RegisterBankClass & GLOBAL;
        Reg32_3 * WINREGS;
        Reg32_3 * ASR;
        Alias & FP;
        Alias & LR;
        Alias & SP;
        Alias & PCR;
        Alias * REGS;
        bool & instrExecuting;
        sc_event & instrEndEvent;
        int routineEntryState;
        int routineExitState;
        unsigned int exitValue;
        std::vector< std::vector< std::string > > routineEntrySequence;
        std::vector< std::vector< std::string > > routineExitSequence;

        public:
        LEON3_ABIIf( unsigned int & PROGRAM_LIMIT, MemoryInterface & dataMem, Reg32_0 & PSR, \
            Reg32_1 & WIM, Reg32_2 & TBR, Reg32_3 & Y, Reg32_3 & PC, Reg32_3 & NPC, RegisterBankClass \
            & GLOBAL, Reg32_3 * WINREGS, Reg32_3 * ASR, Alias & FP, Alias & LR, Alias & SP, Alias \
            & PCR, Alias * REGS, bool & instrExecuting, sc_event & instrEndEvent );

        bool is_little_endian() const throw();
        int get_processor_id() const throw();
        bool is_executing_instr() const throw();
        void wait_instr_end() const throw();
        void pre_call() throw();
        void post_call() throw();
        void return_from_call() throw();
        bool is_routine_entry( const InstructionBase * instr ) throw();
        bool is_routine_exit( const InstructionBase * instr ) throw();
        unsigned char * get_state() const throw();
        void set_state( unsigned char * state ) throw();
        void set_exit_value(unsigned int value) throw();
        unsigned int get_exit_value() throw();
        unsigned int get_code_limit();
        unsigned int read_LR() const throw();
        void set_LR( const unsigned int & newValue ) throw();
        unsigned int read_PC() const throw();
        void set_PC( const unsigned int & newValue ) throw();
        unsigned int read_SP() const throw();
        void set_SP( const unsigned int & newValue ) throw();
        unsigned int read_FP() const throw();
        void set_FP( const unsigned int & newValue ) throw();
        unsigned int read_return_value() const throw();
        void set_return_value( const unsigned int & newValue ) throw();
        std::vector< unsigned int > read_args() const throw();
        void set_args( const std::vector< unsigned int > & args ) throw();
        unsigned int read_gdb_reg( const unsigned int & gdbId ) const throw();
        unsigned int num_gdb_regs() const throw();
        void set_gdb_reg( const unsigned int & newValue, const unsigned int & gdbId ) throw();
        unsigned int read_mem( const unsigned int & address );
        unsigned char read_char_mem( const unsigned int & address );
        void write_mem( const unsigned int & address, unsigned int datum );
        void write_char_mem( const unsigned int & address, unsigned char datum );
        MemoryInterface& get_data_memory();
        virtual ~LEON3_ABIIf();
    };

};



#endif
