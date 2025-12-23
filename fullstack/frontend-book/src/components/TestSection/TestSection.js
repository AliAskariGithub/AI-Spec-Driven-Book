import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import styles from './TestSection.module.css';

// Icon components to replace emojis
const Icons = {
  Target: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2"/></svg>,
  Chart: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M3 3v18h18"/><path d="M18 17V9"/><path d="M13 17V5"/><path d="M8 17v-3"/></svg>,
  Clock: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="10"/><path d="M12 6v6l4 2"/></svg>,
  Check: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="20 6 9 17 4 12"/></svg>,
  Bookmark: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M19 21l-7-5-7 5V5a2 2 0 0 1 2-2h10a2 2 0 0 1 2 2z"/></svg>,
  BookmarkFilled: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="currentColor" stroke="currentColor" strokeWidth="2"><path d="M19 21l-7-5-7 5V5a2 2 0 0 1 2-2h10a2 2 0 0 1 2 2z"/></svg>,
  Moon: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/></svg>,
  Sun: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="5"/><line x1="12" y1="1" x2="12" y2="3"/><line x1="12" y1="21" x2="12" y2="23"/><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"/><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"/><line x1="1" y1="12" x2="3" y2="12"/><line x1="21" y1="12" x2="23" y2="12"/><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"/><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"/></svg>,
  X: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/></svg>,
  Trophy: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M6 9H4.5a2.5 2.5 0 0 1 0-5H6"/><path d="M18 9h1.5a2.5 2.5 0 0 0 0-5H18"/><path d="M4 22h16"/><path d="M10 14.66V17c0 .55-.47.98-.97 1.21C7.85 18.75 7 20.24 7 22"/><path d="M14 14.66V17c0 .55.47.98.97 1.21C16.15 18.75 17 20.24 17 22"/><path d="M18 2H6v7a6 6 0 0 0 12 0V2Z"/></svg>,
  Star: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/></svg>,
  ThumbsUp: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M14 9V5a3 3 0 0 0-3-3l-4 9v11h11.28a2 2 0 0 0 2-1.7l1.38-9a2 2 0 0 0-2-2.3zM7 22H4a2 2 0 0 1-2-2v-7a2 2 0 0 1 2-2h3"/></svg>,
  Book: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20"/><path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z"/></svg>,
  Bulb: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M12 2v2"/><path d="M12 20v2"/><path d="m4.93 4.93 1.41 1.41"/><path d="m17.66 17.66 1.41 1.41"/><path d="M2 12h2"/><path d="M20 12h2"/><path d="m6.34 17.66-1.41 1.41"/><path d="m19.07 4.93-1.41 1.41"/><path d="M9 16v-4.586a2 2 0 0 1 .586-1.414l1.828-1.828A4 4 0 0 0 12 5.828V4"/><path d="M15 16v-4.586a2 2 0 0 0-.586-1.414l-1.828-1.828A4 4 0 0 1 12 5.828V4"/></svg>,
  Party: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M12 2L2 7l10 5 10-5-10-5z"/><path d="M2 17l10 5 10-5"/><path d="M2 12l10 5 10-5"/></svg>,
  Refresh: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="23 4 23 10 17 10"/><polyline points="1 20 1 14 7 14"/><path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15"/></svg>,
  Printer: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="6 9 6 2 18 2 18 9"/><path d="M6 18H4a2 2 0 0 1-2-2v-5a2 2 0 0 1 2-2h16a2 2 0 0 1 2 2v5a2 2 0 0 1-2 2h-2"/><rect x="6" y="14" width="12" height="8"/></svg>,
  Download: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/></svg>,
  ChevronDown: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="6 9 12 15 18 9"/></svg>,
  ChevronLeft: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="15 18 9 12 15 6"/></svg>,
  ChevronRight: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="9 18 15 12 9 6"/></svg>,
  FileText: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/><polyline points="14 2 14 8 20 8"/><line x1="16" y1="13" x2="8" y2="13"/><line x1="16" y1="17" x2="8" y2="17"/><polyline points="10 9 9 9 8 9"/></svg>,
  AlertCircle: () => <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>,
};

const TestSection = ({ questions = [] }) => {
  const optionsContainerRef = useRef(null);
  const feedbackRef = useRef(null);
  
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [selectedOptions, setSelectedOptions] = useState(Array(questions.length).fill(null));
  const [showResults, setShowResults] = useState(false);
  const [showFeedback, setShowFeedback] = useState(false);
  const [timeRemaining, setTimeRemaining] = useState(null);
  const [testStarted, setTestStarted] = useState(false);
  const [timerEnabled, setTimerEnabled] = useState(true);
  const [bookmarkedQuestions, setBookmarkedQuestions] = useState([]);
  const [theme, setTheme] = useState('dark');
  const [animationEnabled, setAnimationEnabled] = useState(true);
  const [confettiActive, setConfettiActive] = useState(false);

  // Apply theme
  useEffect(() => {
    if (typeof document !== 'undefined') {
      document.documentElement.setAttribute('data-theme', theme);
    }
  }, [theme]);

  // Trigger confetti on correct answer
  useEffect(() => {
    if (showFeedback && 
        selectedOptions[currentQuestionIndex] === questions[currentQuestionIndex]?.correct) {
      setConfettiActive(true);
      const timer = setTimeout(() => setConfettiActive(false), 2000);
      return () => clearTimeout(timer);
    }
  }, [showFeedback, selectedOptions, currentQuestionIndex, questions]);

  // Scroll to feedback
  useEffect(() => {
    if (showFeedback && feedbackRef.current) {
      feedbackRef.current.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    }
  }, [showFeedback]);

  // Options entrance animation
  useEffect(() => {
    if (optionsContainerRef.current && !showFeedback && animationEnabled) {
      const options = optionsContainerRef.current.querySelectorAll(`.${styles.option}`);
      options.forEach((option, index) => {
        option.style.opacity = '0';
        option.style.transform = 'translateY(20px)';
        const timer = setTimeout(() => {
          option.style.transition = 'all 0.3s ease';
          option.style.opacity = '1';
          option.style.transform = 'translateY(0)';
        }, 100 * index);
        return () => clearTimeout(timer);
      });
    }
  }, [currentQuestionIndex, showFeedback, animationEnabled]);

  // Calculate score
  const { score, percentage } = useMemo(() => {
    const calculatedScore = selectedOptions.reduce((acc, selected, index) => {
      return acc + (selected === questions[index]?.correct ? 1 : 0);
    }, 0);
    
    return {
      score: calculatedScore,
      percentage: questions.length ? Math.round((calculatedScore / questions.length) * 100) : 0
    };
  }, [selectedOptions, questions]);

  // Timer
  useEffect(() => {
    if (testStarted && timerEnabled && timeRemaining > 0 && !showResults && !showFeedback) {
      const timer = setTimeout(() => {
        setTimeRemaining(prev => prev - 1);
      }, 1000);
      return () => clearTimeout(timer);
    } else if (timeRemaining === 0 && !showFeedback && timerEnabled) {
      handleTimeout();
    }
  }, [timeRemaining, testStarted, showResults, showFeedback, timerEnabled]);

  const handleTimeout = useCallback(() => {
    setShowFeedback(true);
    if (selectedOptions[currentQuestionIndex] === null) {
      setSelectedOptions(prev => {
        const newOptions = [...prev];
        newOptions[currentQuestionIndex] = -1;
        return newOptions;
      });
    }
  }, [currentQuestionIndex, selectedOptions]);

  const startTest = useCallback(() => {
    setTestStarted(true);
    setTimeRemaining(30);
  }, []);

  const toggleTimer = useCallback(() => {
    setTimerEnabled(prev => !prev);
  }, []);

  const toggleTheme = useCallback(() => {
    setTheme(prev => prev === 'light' ? 'dark' : 'light');
  }, []);

  const toggleAnimations = useCallback(() => {
    setAnimationEnabled(prev => !prev);
  }, []);

  const toggleBookmark = useCallback(() => {
    setBookmarkedQuestions(prev => {
      if (prev.includes(currentQuestionIndex)) {
        return prev.filter(idx => idx !== currentQuestionIndex);
      } else {
        return [...prev, currentQuestionIndex];
      }
    });
  }, [currentQuestionIndex]);

  const handleOptionSelect = useCallback((questionIndex, optionIndex) => {
    if (showFeedback) return;
    
    setSelectedOptions(prev => {
      const newOptions = [...prev];
      newOptions[questionIndex] = optionIndex;
      return newOptions;
    });
    setShowFeedback(true);
  }, [showFeedback]);

  const handleNextQuestion = useCallback(() => {
    if (currentQuestionIndex < questions.length - 1) {
      setCurrentQuestionIndex(prev => prev + 1);
      setShowFeedback(false);
      if (timerEnabled) setTimeRemaining(30);
    } else {
      setShowResults(true);
    }
  }, [currentQuestionIndex, questions.length, timerEnabled]);

  const handlePreviousQuestion = useCallback(() => {
    if (currentQuestionIndex > 0) {
      setCurrentQuestionIndex(prev => prev - 1);
      setShowFeedback(selectedOptions[currentQuestionIndex - 1] !== null);
    }
  }, [currentQuestionIndex, selectedOptions]);

  const jumpToQuestion = useCallback((index) => {
    setCurrentQuestionIndex(index);
    setShowFeedback(selectedOptions[index] !== null);
  }, [selectedOptions]);

  const resetTest = useCallback(() => {
    setCurrentQuestionIndex(0);
    setSelectedOptions(Array(questions.length).fill(null));
    setShowResults(false);
    setShowFeedback(false);
    setTestStarted(false);
    setTimeRemaining(null);
    setBookmarkedQuestions([]);
  }, [questions.length]);

  const { performanceMessage, PerformanceIcon } = useMemo(() => {
    if (percentage === 100) {
      return { 
        performanceMessage: 'Perfect Score! Excellent work!',
        PerformanceIcon: Icons.Trophy
      };
    } else if (percentage >= 80) {
      return {
        performanceMessage: 'Great job! Strong understanding!',
        PerformanceIcon: Icons.Star
      };
    } else if (percentage >= 60) {
      return {
        performanceMessage: 'Good effort! Keep practicing!',
        PerformanceIcon: Icons.ThumbsUp
      };
    } else {
      return {
        performanceMessage: 'Review recommended. Keep learning!',
        PerformanceIcon: Icons.Book
      };
    }
  }, [percentage]);

  const Confetti = () => (
    <div className={styles.confettiContainer}>
      {Array.from({ length: 50 }).map((_, index) => (
        <div
          key={index}
          className={styles.confettiPiece}
          style={{
            left: `${Math.random() * 100}%`,
            animationDelay: `${Math.random() * 0.5}s`,
            backgroundColor: `hsl(${Math.random() * 360}, 100%, 50%)`
          }}
        />
      ))}
    </div>
  );

  if (!questions || questions.length === 0) {
    return (
      <div className={styles.testSection}>
        <div className={styles.emptyState}>
          <Icons.FileText />
          <p>No questions available for this test.</p>
        </div>
      </div>
    );
  }

  // Start screen continues...
  if (!testStarted) {
    return (
      <div className={styles.testSection}>
        <div className={`${styles.startScreen} ${animationEnabled ? styles.animatedBg : ''}`}>
          <div className={styles.themeToggle}>
            <button onClick={toggleTheme} className={styles.themeButton} aria-label="Toggle theme">
              {theme === 'light' ? <Icons.Moon /> : <Icons.Sun />}
            </button>
          </div>
          <div className={`${styles.startIcon} ${animationEnabled ? styles.animated : ''}`}>
            <Icons.Target />
          </div>
          <h3 className={styles.startTitle}>Ready to Test Your Knowledge?</h3>
          <div className={styles.testInfo}>
            <div className={`${styles.infoItem} ${animationEnabled ? styles.fadeInUp : ''}`}>
              <Icons.Chart />
              <span>{questions.length} Questions</span>
            </div>
            <div className={`${styles.infoItem} ${animationEnabled ? styles.fadeInUp : ''}`}>
              <Icons.Clock />
              <span>30s per question</span>
            </div>
            <div className={`${styles.infoItem} ${animationEnabled ? styles.fadeInUp : ''}`}>
              <Icons.Check />
              <span>Instant feedback</span>
            </div>
            <div className={`${styles.infoItem} ${animationEnabled ? styles.fadeInUp : ''}`}>
              <Icons.Bookmark />
              <span>Bookmark questions</span>
            </div>
          </div>
          <div className={`${styles.settingsContainer} ${animationEnabled ? styles.fadeInUp : ''}`}>
            <label className={styles.settingItem}>
              <input 
                type="checkbox" 
                checked={timerEnabled}
                onChange={toggleTimer}
                className={styles.settingCheckbox}
              />
              <span className={styles.settingLabel}>Enable timer</span>
            </label>
            <label className={styles.settingItem}>
              <input 
                type="checkbox" 
                checked={animationEnabled}
                onChange={toggleAnimations}
                className={styles.settingCheckbox}
              />
              <span className={styles.settingLabel}>Enable animations</span>
            </label>
          </div>
          <button 
            onClick={startTest} 
            className={`${styles.startButton} ${animationEnabled ? styles.pulseButton : ''}`}
          >
            {selectedOptions.some(option => option !== null) ? 'Continue Test' : 'Start Test'}
          </button>
          {selectedOptions.some(option => option !== null) && (
            <button onClick={resetTest} className={styles.resetButton}>
              Reset Progress
            </button>
          )}
        </div>
      </div>
    );
  }

  // Results screen - will continue in next message due to length
  if (showResults) {
    const answeredQuestions = selectedOptions.filter(option => option !== null && option !== -1).length;
    const correctQuestions = selectedOptions.filter((option, index) => option === questions[index]?.correct).length;
    const incorrectQuestions = answeredQuestions - correctQuestions;
    const skippedQuestions = questions.length - answeredQuestions;
    
    return (
      <div className={styles.testSection}>
        <div className={`${styles.results} ${animationEnabled ? styles.fadeIn : ''}`}>
          <div className={styles.themeToggle}>
            <button onClick={toggleTheme} className={styles.themeButton} aria-label="Toggle theme">
              {theme === 'light' ? <Icons.Moon /> : <Icons.Sun />}
            </button>
          </div>
          
          <div className={`${styles.resultsHeader} ${animationEnabled ? styles.fadeInUp : ''}`}>
            <div className={styles.performanceEmoji}><PerformanceIcon /></div>
            <h3 className={styles.resultsTitle}>Test Complete!</h3>
            <div className={`${styles.scoreCircle} ${percentage >= 80 ? styles.highScore : percentage >= 60 ? styles.mediumScore : styles.lowScore}`}>
              <svg className={styles.scoreRing} viewBox="0 0 100 100">
                <circle className={styles.scoreBackground} cx="50" cy="50" r="40" />
                <circle 
                  className={styles.scoreProgress} 
                  cx="50" 
                  cy="50" 
                  r="40" 
                  style={{
                    strokeDasharray: `${percentage * 2.51} 251`,
                    strokeDashoffset: "0"
                  }}
                />
              </svg>
              <div className={styles.scoreContent}>
                <div className={styles.scoreValue}>{percentage}%</div>
                <div className={styles.scoreLabel}>
                  {score} / {questions.length}
                </div>
              </div>
            </div>
            <p className={styles.performance}>{performanceMessage}</p>
          </div>

          <div className={styles.resultsActions}>
            <button onClick={resetTest} className={styles.retakeButton}>
              <Icons.Refresh />
              <span>Retake Test</span>
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
  const isBookmarked = bookmarkedQuestions.includes(currentQuestionIndex);

  return (
    <div className={styles.testSection}>
      {confettiActive && <Confetti />}
      
      <div className={styles.themeToggle}>
        <button onClick={toggleTheme} className={styles.themeButton} aria-label="Toggle theme">
          {theme === 'light' ? <Icons.Moon /> : <Icons.Sun />}
        </button>
      </div>
      
      <div className={styles.progressBarContainer}>
        <div className={styles.progressBar}>
          <div 
            className={styles.progressFill} 
            style={{ width: `${progressPercentage}%` }}
          />
        </div>
        <div className={styles.progressText}>{Math.round(progressPercentage)}%</div>
      </div>

      <div className={styles.questionHeader}>
        <div className={styles.questionCounter}>
          Question {currentQuestionIndex + 1} of {questions.length}
        </div>
        <div className={styles.questionActions}>
          {timerEnabled && timeRemaining !== null && (
            <div className={`${styles.timer} ${timeRemaining <= 10 ? styles.timerWarning : ''}`}>
              <Icons.Clock />
              <span className={styles.timerValue}>{timeRemaining}s</span>
            </div>
          )}
          <button 
            className={`${styles.bookmarkButton} ${isBookmarked ? styles.bookmarked : ''}`}
            onClick={toggleBookmark}
            aria-label={isBookmarked ? "Remove bookmark" : "Bookmark question"}
          >
            {isBookmarked ? <Icons.BookmarkFilled /> : <Icons.Bookmark />}
          </button>
        </div>
      </div>

      <div className={`${styles.question} ${animationEnabled ? styles.fadeIn : ''}`}>
        <h3 className={styles.questionText}>{currentQuestion.question}</h3>

        <div className={styles.options} ref={optionsContainerRef}>
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
                {showCorrect && <Icons.Check />}
                {showIncorrect && <Icons.X />}
              </button>
            );
          })}
        </div>

        {showFeedback && (
          <div 
            ref={feedbackRef}
            className={`${styles.feedback} ${
              userAnswer === currentQuestion.correct ? styles.feedbackCorrect : styles.feedbackIncorrect
            }`}
          >
            <div className={styles.feedbackIcon}>
              {userAnswer === currentQuestion.correct ? <Icons.Party /> : <Icons.Bulb />}
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
            <Icons.ChevronLeft /> Previous
          </button>
          
          <button
            onClick={handleNextQuestion}
            disabled={userAnswer === null}
            className={styles.nextButton}
          >
            {currentQuestionIndex < questions.length - 1 ? (
              <>Next <Icons.ChevronRight /></>
            ) : (
              'Show Results'
            )}
          </button>
        </div>
      </div>
    </div>
  );
};

export default React.memo(TestSection);