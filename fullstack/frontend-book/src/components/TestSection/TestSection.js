import React, { useState, useEffect } from 'react';
import styles from './TestSection.module.css';

const TestSection = ({ questions = [] }) => {
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedOptions, setSelectedOptions] = useState(
    Array(questions.length).fill(null)
  );
  const [showResults, setShowResults] = useState(false);
  const [showFeedback, setShowFeedback] = useState(false);
  const [timeRemaining, setTimeRemaining] = useState(null);
  const [testStarted, setTestStarted] = useState(false);

  // Timer functionality (optional - 30 seconds per question)
  useEffect(() => {
    if (testStarted && timeRemaining > 0 && !showResults && !showFeedback) {
      const timer = setTimeout(() => {
        setTimeRemaining(timeRemaining - 1);
      }, 1000);
      return () => clearTimeout(timer);
    } else if (timeRemaining === 0 && !showFeedback) {
      handleTimeout();
    }
  }, [timeRemaining, testStarted, showResults, showFeedback]);

  const handleTimeout = () => {
    setShowFeedback(true);
  };

  const startTest = () => {
    setTestStarted(true);
    setTimeRemaining(30); // 30 seconds per question
  };

  const handleOptionSelect = (questionIndex, optionIndex) => {
    if (showFeedback) return; // Prevent changing answer after feedback shown
    
    const newSelectedOptions = [...selectedOptions];
    newSelectedOptions[questionIndex] = optionIndex;
    setSelectedOptions(newSelectedOptions);
    setShowFeedback(true);
  };

  const handleNextQuestion = () => {
    if (currentQuestionIndex < questions.length - 1) {
      setCurrentQuestionIndex(currentQuestionIndex + 1);
      setShowFeedback(false);
      setTimeRemaining(30); // Reset timer for next question
    } else {
      setShowResults(true);
    }
  };

  const handlePreviousQuestion = () => {
    if (currentQuestionIndex > 0) {
      setCurrentQuestionIndex(currentQuestionIndex - 1);
      setShowFeedback(selectedOptions[currentQuestionIndex - 1] !== null);
    }
  };

  const resetTest = () => {
    setCurrentQuestionIndex(0);
    setSelectedOptions(Array(questions.length).fill(null));
    setShowResults(false);
    setShowFeedback(false);
    setTestStarted(false);
    setTimeRemaining(null);
  };

  // Validation
  if (!questions || questions.length === 0) {
    return (
      <div className={styles.testSection}>
        <div className={styles.emptyState}>
          <span className={styles.emptyIcon}>📝</span>
          <p>No questions available for this test.</p>
        </div>
      </div>
    );
  }

  // Start screen
  if (!testStarted) {
    return (
      <div className={styles.testSection}>
        <div className={styles.startScreen}>
          <div className={styles.startIcon}>🎯</div>
          <h3 className={styles.startTitle}>Ready to Test Your Knowledge?</h3>
          <div className={styles.testInfo}>
            <div className={styles.infoItem}>
              <span className={styles.infoIcon}>📊</span>
              <span>{questions.length} Questions</span>
            </div>
            <div className={styles.infoItem}>
              <span className={styles.infoIcon}>⏱️</span>
              <span>30s per question</span>
            </div>
            <div className={styles.infoItem}>
              <span className={styles.infoIcon}>✅</span>
              <span>Instant feedback</span>
            </div>
          </div>
          <button onClick={startTest} className={styles.startButton}>
            Start Test
          </button>
        </div>
      </div>
    );
  }

  // Results screen
  if (showResults) {
    const score = selectedOptions.reduce((acc, selected, index) => {
      return acc + (selected === questions[index].correct ? 1 : 0);
    }, 0);
    
    const percentage = Math.round((score / questions.length) * 100);

    let performanceMessage = '';
    let performanceEmoji = '';
    if (percentage === 100) {
      performanceMessage = 'Perfect Score! Excellent work!';
      performanceEmoji = '🏆';
    } else if (percentage >= 80) {
      performanceMessage = 'Great job! Strong understanding!';
      performanceEmoji = '🌟';
    } else if (percentage >= 60) {
      performanceMessage = 'Good effort! Keep practicing!';
      performanceEmoji = '👍';
    } else {
      performanceMessage = 'Review recommended. Keep learning!';
      performanceEmoji = '📚';
    }

    return (
      <div className={styles.testSection}>
        <div className={styles.results}>
          <div className={styles.resultsHeader}>
            <div className={styles.performanceEmoji}>{performanceEmoji}</div>
            <h3 className={styles.resultsTitle}>Test Complete!</h3>
            <div className={styles.scoreCircle}>
              <div className={styles.scoreValue}>{percentage}%</div>
              <div className={styles.scoreLabel}>
                {score} / {questions.length}
              </div>
            </div>
            <p className={styles.performance}>{performanceMessage}</p>
          </div>

          <div className={styles.detailedResults}>
            <h4 className={styles.feedbackTitle}>
              <span className={styles.feedbackIcon}>💡</span>
              Detailed Feedback
            </h4>
            {questions.map((question, index) => {
              const isCorrect = selectedOptions[index] === question.correct;
              const wasAnswered = selectedOptions[index] !== null;
              
              return (
                <div 
                  key={index} 
                  className={`${styles.questionResult} ${
                    isCorrect ? styles.resultCorrect : styles.resultIncorrect
                  }`}
                >
                  <div className={styles.resultHeader}>
                    <span className={styles.questionNumber}>Q{index + 1}</span>
                    <span className={isCorrect ? styles.correctBadge : styles.incorrectBadge}>
                      {isCorrect ? '✓ Correct' : wasAnswered ? '✗ Incorrect' : '⊘ Skipped'}
                    </span>
                  </div>
                  <p className={styles.resultQuestion}>{question.question}</p>
                  
                  {wasAnswered && (
                    <div className={styles.answerComparison}>
                      <div className={styles.yourAnswer}>
                        <strong>Your answer:</strong>
                        <span className={isCorrect ? styles.correctText : styles.incorrectText}>
                          {question.options[selectedOptions[index]]}
                        </span>
                      </div>
                      {!isCorrect && (
                        <div className={styles.correctAnswer}>
                          <strong>Correct answer:</strong>
                          <span className={styles.correctText}>
                            {question.options[question.correct]}
                          </span>
                        </div>
                      )}
                    </div>
                  )}
                  
                  <div className={styles.explanation}>
                    <strong>Explanation:</strong> {question.explanation}
                  </div>
                </div>
              );
            })}
          </div>

          <div className={styles.resultsActions}>
            <button onClick={resetTest} className={styles.retakeButton}>
              <span className={styles.buttonIcon}>🔄</span>
              Retake Test
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Question screen
  const currentQuestion = questions[currentQuestionIndex];
  const userAnswer = selectedOptions[currentQuestionIndex];
  const progressPercentage = ((currentQuestionIndex + 1) / questions.length) * 100;

  return (
    <div className={styles.testSection}>
      <div className={styles.progressBar}>
        <div 
          className={styles.progressFill} 
          style={{ width: `${progressPercentage}%` }}
        />
      </div>

      <div className={styles.questionHeader}>
        <div className={styles.questionCounter}>
          Question {currentQuestionIndex + 1} of {questions.length}
        </div>
        {timeRemaining !== null && (
          <div className={`${styles.timer} ${timeRemaining <= 10 ? styles.timerWarning : ''}`}>
            <span className={styles.timerIcon}>⏱️</span>
            <span className={styles.timerValue}>{timeRemaining}s</span>
          </div>
        )}
      </div>

      <div className={styles.question}>
        <h3 className={styles.questionText}>{currentQuestion.question}</h3>

        <div className={styles.options}>
          {currentQuestion.options.map((option, index) => {
            const isSelected = userAnswer === index;
            const isCorrect = index === currentQuestion.correct;
            const showCorrect = showFeedback && isCorrect;
            const showIncorrect = showFeedback && isSelected && !isCorrect;
            
            return (
              <button
                key={index}
                className={`${styles.option} ${
                  showCorrect ? styles.correctOption :
                  showIncorrect ? styles.incorrectOption :
                  isSelected ? styles.selectedOption :
                  ''
                } ${showFeedback ? styles.disabled : ''}`}
                onClick={() => handleOptionSelect(currentQuestionIndex, index)}
                disabled={showFeedback}
              >
                <span className={styles.optionLetter}>
                  {String.fromCharCode(65 + index)}
                </span>
                <span className={styles.optionText}>{option}</span>
                {showCorrect && <span className={styles.checkMark}>✓</span>}
                {showIncorrect && <span className={styles.crossMark}>✗</span>}
              </button>
            );
          })}
        </div>

        {showFeedback && (
          <div className={`${styles.feedback} ${
            userAnswer === currentQuestion.correct ? styles.feedbackCorrect : styles.feedbackIncorrect
          }`}>
            <div className={styles.feedbackIcon}>
              {userAnswer === currentQuestion.correct ? '🎉' : '💡'}
            </div>
            <div className={styles.feedbackContent}>
              <p className={styles.feedbackTitle}>
                {userAnswer === currentQuestion.correct ? 'Correct!' : 'Not quite right'}
              </p>
              <p className={styles.explanation}>{currentQuestion.explanation}</p>
            </div>
          </div>
        )}

        <div className={styles.navigation}>
          <button
            onClick={handlePreviousQuestion}
            disabled={currentQuestionIndex === 0}
            className={styles.prevButton}
          >
            ← Previous
          </button>
          
          <button
            onClick={handleNextQuestion}
            disabled={userAnswer === null}
            className={styles.nextButton}
          >
            {currentQuestionIndex < questions.length - 1 ? 'Next →' : 'Show Results'}
          </button>
        </div>
      </div>
    </div>
  );
};

export default TestSection;